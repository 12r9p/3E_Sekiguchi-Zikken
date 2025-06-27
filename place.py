# import DobotDLL as dType
import time

"""

動作確認済み

Dobot Magician – 色分別ロボットアーム制御スクリプト（完全版）
===================================================================
このスクリプトは 15 mm 角ブロックをカラーセンサで識別し，
Dobot Magician が 2 列（列0 = 青→緑→赤，列1 = 青→赤）に
自動で積み上げるデモです。

* **把持** と **センサ移動** は高速な **MOVJ(PTP)**。
* **配置／格納／取り出し** は干渉回避のため **MOVL(直線補間)**。
* パラメータは `CONFIG` に一元化。速度や座標を現場に合わせて調整。

動作シーケンス
----------------
1. `wait_for_block`   : 把持位置上空でフォトセンサ待機
2. `pick_block`      : 降下→吸着→安全高さへ上昇
3. `measure_color`   : センサ上空へ水平移動→降下→RGB 取得
4. `place / stash`   : 置き列に直接プレース or バッファ格納
5. `flush`           : バッファから必要な色を回収して列に補充
"""

# ==================================================
# CONFIG
# ==================================================
CONFIG = {
    # --- 座標 (mm) -----------------------------------------------
    'grab_pos':    {'x': 263, 'y': 165,  'z': 16},   # ブロック停止位置中心
    'sensor_pos':  {'x': 193, 'y': 111, 'z': 25},   # カラーセンサ直上
    'buffer_base': {                                         # 色別バッファ起点
        'R': {'x': 300.0, 'y': -65.0, 'z': -42.0},
        'G': {'x': 260.0, 'y': -65.0, 'z': -42.0},
        'B': {'x': 220.0, 'y': -65.0, 'z': -42.0},
    },
    'place_base':  {'x': 200.0, 'y': -150.0, 'z': -42.0},  # 完成列起点

    # --- 幾何パラメータ -------------------------------------------
    'buffer_interval_y': 50.0,  # バッファ列間中心距離
    'buffer_interval_z': 24,  # ブロック高さ
    'place_interval_y':  45.0,  # 積み列間隔
    'place_interval_z':  24,  # ブロック高さ

    'clearance_z':        50,   # XY 移動時の安全高さ
    'senser_clearance_z': 10,   # コンベアからセンサーまでの移動高さ
    'approach_offset_z':  10.0, # 把持前に +Z 待機する量

    # --- 速度 (% 指定) --------------------------------------------
    'ptp_vel_pct': 500,  # MOVJ 速度
    'ptp_acc_pct': 500,  # MOVJ 加速度
    'cp_vel_pct':  500,  # MOVL 速度
    'cp_acc_pct':  500,  # MOVL 加速度

    # --- タイミング ------------------------------------------------
    'ir_pause': 0.05,   # フォトセンサ反応後の待機 [s]
}
C = CONFIG  # ショートネーム

# ==================================================
# 実行時に変化するグローバル状態
# ==================================================
buffer_cnt = {'R': 0, 'G': 0, 'B': 0}   # 色別バッファ在庫数
place_cnt  = [0, 0]                      # 列ごとの積層段数
NEXT_SEQ   = [['B', 'G', 'R'],           # 列0 の目標シーケンス
              ['B', 'R']]               # 列1 の目標シーケンス
all_columns_completed_flag = False # New global flag
completed_units_count = 0 # New global counter


# ==================================================
# Dobot 初期化
# ==================================================

def init_dobot(api):
    dType.SetEndEffectorParamsEx(api, 59.7, 0, 0, 1)
    dType.SetColorSensor(api, 1, 2, 1)
    dType.SetInfraredSensor(api, 1, 1, 1)
    time.sleep(0.2)
    dType.SetPTPCommonParamsEx(api, C['ptp_vel_pct'], C['ptp_acc_pct'], 1)
    try:
        dType.SetCPParamsEx(api, C['cp_vel_pct'], C['cp_acc_pct'], 1)
    except AttributeError:
        pass

# ==================================================
# 低レベルユーティリティ
# ==================================================

def movj(api, x, y, z):
    dType.SetPTPCmdEx(api, 0, x, y, z, 0, 1)

def movl(api, x, y, z):
    dType.SetPTPCmdEx(api, 2, x, y, z, 0, 1)

def lift_to_clearance(api):
    pose = dType.GetPose(api)
    if abs(pose[2] - C['clearance_z']) > 0.05:
        movl(api, pose[0], pose[1], C['clearance_z'])

def lift_to_senser_clearance(api):
    pose = dType.GetPose(api)
    if abs(pose[2] - C['senser_clearance_z']) > 0.05:
        movl(api, pose[0], pose[1], C['senser_clearance_z'])

def suction(api, on):
    dType.SetEndEffectorSuctionCupEx(api, 1 if on else 0, 1)

# ==================================================
# ブロック検出→把持
# ==================================================

def wait_for_block(api):
    # Assumes robot is already at gp['x'], gp['y'] at gp['z'] + C['approach_offset_z']
    while not dType.GetInfraredSensor(api, 1)[0]:
        time.sleep(0.01)
    time.sleep(C['ir_pause'])

def pick_block(api):
    gp = C['grab_pos']
    movl(api, gp['x'], gp['y'], gp['z'])
    suction(api, True)
    time.sleep(0.1)
    lift_to_senser_clearance(api)

# ==================================================
# 色判定
# ==================================================

def measure_color(api):
    sp = C['sensor_pos']
    # Assumes robot is already at sp['x'], sp['y'] at C['senser_clearance_z']
    if abs(sp['z'] - C['senser_clearance_z']) > 0.05:
        movl(api, sp['x'], sp['y'], sp['z'])
    time.sleep(0.3)
    rgb = [dType.GetColorSensorEx(api, i) for i in range(3)]
    time.sleep(0.3)
    lift_to_clearance(api)
    return ['R', 'G', 'B'][rgb.index(max(rgb))]

# ==================================================
# 座標計算
# ==================================================

def buffer_xyz(color, idx):
    col, layer = divmod(idx, 3)
    b = C['buffer_base'][color]
    return b['x'], b['y'] - col*C['buffer_interval_y'], b['z'] + layer*C['buffer_interval_z']

def place_xyz(col, layer):
    p = C['place_base']
    return p['x'], p['y'] - col*C['place_interval_y'], p['z'] + layer*C['place_interval_z']

# ==================================================
# バッファ／配置オペレーション
# ==================================================

def stash(api, color):
    """置けない色をバッファに格納 (MOVL)。"""
    # Assumes robot is already at buffer_xyz(color, buffer_cnt[color]) at clearance_z
    movl(api, *buffer_xyz(color, buffer_cnt[color]))
    suction(api, False)
    time.sleep(0.08)
    buffer_cnt[color] += 1
    lift_to_clearance(api)

def pull(api, color):
    """必要色をバッファから取り出し (MOVL)。"""
    # Assumes robot is already at buffer_xyz(color, buffer_cnt[color]-1) at clearance_z
    movl(api, *buffer_xyz(color, buffer_cnt[color]-1))
    suction(api, True)
    time.sleep(0.08)
    buffer_cnt[color] -= 1
    lift_to_clearance(api)

def place(api, color, col):
    """列 col にブロックを配置 (MOVL)。"""
    # Assumes robot is already at place_xyz(col, place_cnt[col]) at clearance_z
    movl(api, *place_xyz(col, place_cnt[col]))
    suction(api, False)
    time.sleep(0.08)
    place_cnt[col] += 1
    lift_to_clearance(api)

# ==================================================
# 9. バッファ掃き出し
# ==================================================

def flush(api):
    """列が次に欲しい色をバッファから補充する。"""
    for col, seq in enumerate(NEXT_SEQ):
        while place_cnt[col] < len(seq):
            need = seq[place_cnt[col]]
            print("  Flush - Column {}: Next need: {}, Buffer count: {}".format(col, need, buffer_cnt[need]))
            # 在庫なければ次の列へ
            if buffer_cnt[need] == 0:
                break
            # バッファから取り出して配置
            target_x, target_y, target_z = buffer_xyz(need, buffer_cnt[need]-1)
            lift_to_clearance(api)
            movl(api, target_x, target_y, C['clearance_z'])
            pull(api, need)

            target_x, target_y, target_z = place_xyz(col, place_cnt[col])
            lift_to_clearance(api)
            movl(api, target_x, target_y, C['clearance_z'])
            place(api, need, col)

# ==================================================
# メインループ
# ==================================================
api = dType.load()
init_dobot(api)
print("\n\n\n\n=== ソート開始 ===")

while True:
    print("ループ開始 - 積層段数: {}、バッファ在庫: {}".format(place_cnt, buffer_cnt))

    # (1) ブロック検出 → 把持
    gp = C['grab_pos']
    print("ブロック停止位置に移動します: x={}, y={}, z={} (approach offset付き)".format(gp['x'], gp['y'], gp['z'] + C['approach_offset_z']))
    lift_to_clearance(api)
    movl(api, gp['x'], gp['y'], gp['z'] + C['approach_offset_z'])
    print("フォトセンサ待機中...")
    wait_for_block(api)
    print("ブロック検出！ 持ち上げ開始")
    pick_block(api)

    # (2) 色判定
    sp = C['sensor_pos']
    print("カラーセンサ位置に移動します: x={}, y={}, z={} (最終目的地)".format(sp['x'], sp['y'], sp['z']))
    lift_to_clearance(api)
    movl(api, sp['x'], sp['y'], C['clearance_z'])
    movl(api, sp['x'], sp['y'], sp['z'])
    color = measure_color(api)
    print("取得色: {}".format(color))

    # (2-1) 完成列リセットチェック
    if all_columns_completed_flag:
        if color == 'B':
            print("全列完成済みかつ 'B' ブロック検出。新しいシーケンスにリセットします。")
            place_cnt = [0] * len(NEXT_SEQ)
            all_columns_completed_flag = False
        elif buffer_cnt['B'] > 0:
            print("全列完成済みかつバッファに 'B' ブロックあり。新しいシーケンスにリセットし、バッファ内の 'B' を利用します。")
            place_cnt = [0] * len(NEXT_SEQ)
            all_columns_completed_flag = False

    # (3) 列に直接置けるかチェック
    placed = False
    for col, seq in enumerate(NEXT_SEQ):
        if place_cnt[col] < len(seq) and seq[place_cnt[col]] == color:
            target_x, target_y, target_z = place_xyz(col, place_cnt[col])
            print("列 {} に {} ブロックを直接配置します: x={}, y={}, z={}".format(col, color, target_x, target_y, C['clearance_z']))
            lift_to_clearance(api)
            movl(api, target_x, target_y, C['clearance_z'])
            place(api, color, col)
            print("列 {} に {} ブロックを配置完了。".format(col, color))
            placed = True
            break

    # (4) 直接配置できなければバッファへ格納
    if not placed:
        target_x, target_y, target_z = buffer_xyz(color, buffer_cnt[color])
        print("直接配置できないため、バッファに {} ブロックを格納します: x={}, y={}, z={}".format(color, target_x, target_y, C['clearance_z']))
        lift_to_clearance(api)
        movl(api, target_x, target_y, C['clearance_z'])
        stash(api, color)
        print("バッファへ {} 格納完了。".format(color))

    # (5) バッファから必要なブロックを排出（フラッシュ）
    print("バッファの内容を確認し、必要なブロックを配置します。")
    flush(api)

    # (6) ユニット完成チェック
    all_columns_completed = True
    for col, seq in enumerate(NEXT_SEQ):
        if place_cnt[col] == len(seq):
            print("完了: 列 {} が完成しました！".format(col))
        else:
            all_columns_completed = False

    if all_columns_completed:
        completed_units_count += 1
        print("全ユニット完成数: {} 個".format(completed_units_count))
        all_columns_completed_flag = True