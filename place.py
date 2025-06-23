import DobotDLL as dType
import time

"""
流れの概要
-----------
1. **待機** ：把持位置 (grab_pos) の Z 座標 +10 mm でフォトセンサ待ち
2. **下降** ：フォトセンサがオンになったら 50 ms 待機 → 把持高さへ降下
3. **把持** ：吸着カップ ON でブロックを吸着
4. **上昇** ：クリアランス高さ (clearance_z) までリフト
5. **水平移動** ：カラーセンサの真上 (clearance_z 面) まで水平移動
6. **色判定** ：センサ高さへ降下し RGB を読み取り → R/G/B を決定
7. **配置 or バッファ** ：
   * 該当列で今すぐ必要な色ならその列へプレース
   * 必要でなければ色別バッファへ一時格納
8. **バッファ掃き出し** ：列が次に必要とする色がバッファにあれば取り出して配置
"""

# -------------------- ユーザ設定 --------------------
CONFIG = {
    # --- 基本座標 ---
    'grab_pos':    {'x': 252.5, 'y': 130.0,  'z': 14.4},   # 把持
    'sensor_pos':  {'x': 191.3, 'y': 112.6, 'z': 25.1},   # カラーセンサ真上
    'buffer_base': {
        'R': {'x': 300.0, 'y': -65.0, 'z': -42.0},
        'G': {'x': 260.0, 'y': -65.0, 'z': -42.0},
        'B': {'x': 220.0, 'y': -65.0, 'z': -42.0},
    },
    'place_base':  {'x': 200.0, 'y': -150.0, 'z': -42.0},

    # --- 間隔・高さ ---
    'buffer_interval_y': 20.0,
    'buffer_interval_z': 15.0,
    'place_interval_y':  45.0,
    'place_interval_z':  15.0,
    'clearance_z':       50.0,   # XY 移動高さ
    'approach_offset_z': 10.0,   # 把持前退避 +Z

    # --- 速度パラメータ (1–100 %) ---
    'ptp_vel_pct': 50,  # PTP 速度
    'ptp_acc_pct': 50,  # PTP 加速度

    # --- タイミング ---
    'ir_pause': 0.05,   # フォトセンサ反応後の待機 [s]
}
C = CONFIG  # 省略用

# ---------- 内部状態 ----------
buffer_cnt = {'R': 0, 'G': 0, 'B': 0}
place_cnt  = [0, 0]
NEXT_SEQ   = [['B','G','R'], ['B','R']]

# ==================================================
#  Dobot 初期化処理
# ==================================================

def init_dobot(api) -> None:
    """Dobot Magician の初期設定。
    CONFIG の `ptp_vel_pct` / `ptp_acc_pct` をそのまま速度に使用。"""
    # エンドエフェクタパラメータ
    dType.SetEndEffectorParamsEx(api, 59.7, 0, 0, 1)
    # センサ有効化
    dType.SetColorSensor(api, 1, 2, 1)
    dType.SetInfraredSensor(api, 1, 1, 1)
    time.sleep(0.3)
    # PTP 共通速度を CONFIG から設定
    dType.SetPTPCommonParamsEx(api, C['ptp_vel_pct'], C['ptp_acc_pct'], 1)

# ==================================================
#  低レベル移動＆吸着
# ==================================================

def move(api, x: float, y: float, z: float) -> None:
    """ワールド座標 (x,y,z) へ PTP 直線移動。"""
    dType.SetPTPCmdEx(api, 0, x, y, z, 0, 1)


def lift_to_clearance(api) -> None:
    """現在 XY を保ったまま `clearance_z` へリフトアップ。"""
    pose = dType.GetPose(api)
    if abs(pose[2] - C['clearance_z']) > 0.05:
        move(api, pose[0], pose[1], C['clearance_z'])


def suction(api, on: bool) -> None:
    """吸着カップの真空ポンプ制御。on=True で ON。"""
    dType.SetEndEffectorSuctionCupEx(api, 1 if on else 0, 1)

# ==================================================
#  ステージ 1: ブロック検出→把持
# ==================================================

def wait_for_block(api) -> None:
    """把持位置上空でフォトセンサを監視し、ブロック到来を待つ。"""
    gp = C['grab_pos']
    wait_z = gp['z'] + C['approach_offset_z']
    move(api, gp['x'], gp['y'], wait_z)
    # フォトセンサ(GP2) が反応するまでポーリング
    while not dType.GetInfraredSensor(api, 1)[0]:
        time.sleep(0.01)
    time.sleep(C['ir_pause'])  # ブロック静止を待つ
    move(api, gp['x'], gp['y'], gp['z'])  # 降下して把持高さへ


def pick_block(api) -> None:
    """ブロックを吸着し、クリアランス高さまで戻る。"""
    suction(api, True)
    time.sleep(0.15)  # 負圧安定
    lift_to_clearance(api)

# ==================================================
#  ステージ 2: 色判定
# ==================================================

def goto_sensor_and_measure(api) -> str:
    """カラーセンサ位置へ移動して RGB を取得し、'R','G','B' を返す。"""
    sp = C['sensor_pos']
    # 水平移動（Z は clearance_z のまま）
    move(api, sp['x'], sp['y'], C['clearance_z'])
    # Z 降下（sensor_pos.z と clearance_z が同じなら移動無し）
    if abs(C['clearance_z'] - sp['z']) > 0.05:
        move(api, sp['x'], sp['y'], sp['z'])
    time.sleep(0.15)
    rgb = [dType.GetColorSensorEx(api, i) for i in range(3)]
    color = ['R','G','B'][rgb.index(max(rgb))]
    lift_to_clearance(api)
    return color

# ==================================================
#  ステージ 3: バッファ／置き列操作
# ==================================================

def buffer_xyz(color: str, idx: int):
    col, layer = divmod(idx, 3)
    b = C['buffer_base'][color]
    return (
        b['x'],
        b['y'] - col * C['buffer_interval_y'],
        b['z'] + layer * C['buffer_interval_z']
    )


def place_xyz(col: int, layer: int):
    p = C['place_base']
    return (
        p['x'],
        p['y'] - col * C['place_interval_y'],
        p['z'] + layer * C['place_interval_z']
    )


def stash(api, color: str) -> None:
    move(api, *buffer_xyz(color, buffer_cnt[color]))
    suction(api, False)
    time.sleep(0.1)
    buffer_cnt[color] += 1
    lift_to_clearance(api)


def pull(api, color: str) -> None:
    move(api, *buffer_xyz(color, buffer_cnt[color]-1))
    suction(api, True)
    time.sleep(0.1)
    buffer_cnt[color] -= 1
    lift_to_clearance(api)


def place(api, color: str, col: int) -> None:
    move(api, *place_xyz(col, place_cnt[col]))
    suction(api, False)
    time.sleep(0.1)
    place_cnt[col] += 1
    lift_to_clearance(api)


def flush(api) -> None:
    for col, seq in enumerate(NEXT_SEQ):
        while place_cnt[col] < len(seq):
            need = seq[place_cnt[col]]
            if buffer_cnt[need] == 0:
                break
            pull(api, need)
            place(api, need, col)

# ==================================================
#  メインループ
# ==================================================

# Dobot API のロードを開始します
api = dType.load()

# Dobot の初期設定を実施する関数を呼び出し
init_dobot(api)

# ソート処理の開始をログ出力で知らせる
print('=== Sorting Start ===')

# メインループ：ブロックの検出、把持、色判定、配置の順に処理を繰り返す
while True:
  # ブロックが来るまでフォトセンサで監視する
  wait_for_block(api)
  # ブロック到着後、吸着してブロックを把持する
  pick_block(api)
  # カラーセンサ位置へ移動し、ブロックの色(R,G,B)を取得する
  color = goto_sensor_and_measure(api)
  # 取得した色をログ出力する
  print(color)
  # ブロックが対応する配置先に既に必要な色があれば置くためのフラグ
  placed = False
  # 配置先（列）のループ。各列の順番で必要な色を確認する
  for col, seq in enumerate(NEXT_SEQ):
    # 対象の列でまだブロックが必要な場合、現在の配置カウントがシーケンスの長さ未満であることを確認
    if place_cnt[col] < len(seq) and seq[place_cnt[col]] == color:
      # 必要な色と一致した場合、ブロックをその列に配置する
      place(api, color, col)
      placed = True
      # 処理が完了したのでループを抜ける
      break
  # どの配置先でもブロックが必要とされていなかった場合
  if not placed:
    # バッファに退避させる処理を行う
    stash(api, color)
  # バッファから必要なブロックを配置先に流し出す処理を実施する
  flush(api)
