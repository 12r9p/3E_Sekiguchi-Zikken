import math
import time

# ──────────── 設定（現地合わせで値をセット） ────────────
CONFIG = {
    # ブロック把持用
    'grab_pos':    {'x': 252.5, 'y': 130, 'z': 14.4},
    # 色判定センサ上空
    'sensor_pos':  {'x': 191.3, 'y': 112.6, 'z': 25.1},
    # 色別バッファの front-right 起点座標
    'buffer_base': {
        'R': {'x': 300, 'y': -65, 'z': -42},
        'G': {'x': 260, 'y': -65, 'z': -42},
        'B': {'x': 220, 'y': -65, 'z': -42},
    },
    # 本置き列の front-right 起点座標
    'place_base':   {'x': 200, 'y': -150, 'z': -42},
    # バッファ列間隔 (Y方向)
    'buffer_interval_y': 20,
    # バッファ積層間隔 (Z方向)
    'buffer_interval_z': 15,
    # 本置き列間隔 (Y方向)
    'place_interval_y': 30,
    # 本置き積層間隔 (Z方向)
    'place_interval_z': 15,
    # XY移動時の安全高度
    'safe_z':       50,
}
# ────────────────────────────────────────────────────────

# ランタイム状態
buffer_cnt  = {'R': 0, 'G': 0, 'B': 0}  # 色別バッファ個数
place_cnt   = [0, 0]                    # 列ごとの積層数
NEXT_SEQ    = [['B', 'G', 'R'], ['B', 'R']]

# =============== Dobot 初期化 ==================

def init_dobot(api):
    dType.SetEndEffectorParamsEx(api, 59.7, 0, 0, 1)
    dType.SetColorSensor(api, 1, 2, 1)
    dType.SetInfraredSensor(api, 1, 1, 1)
    time.sleep(1)
    dType.SetPTPJointParamsEx(api, 600,600,600,600,600,600,600,600, 1)
    dType.SetPTPCommonParamsEx(api, 50, 50, 1)
    dType.SetPTPJumpParamsEx(api, 50, 100, 1)

# =============== 転送ユーティリティ =============

EPS = 0.1  # mm 判定誤差

def at_safe_z(api):
    return abs(dType.GetPose(api)[2] - CONFIG['safe_z']) < EPS


def move_to(api, x, y, z, r=0):
    dType.SetPTPCmdEx(api, 0, x, y, z, r, 1)


def move_xy_then_z(api, x, y, z):
    # ① まだ safe_z でなければその場で Z↑safe
    if not at_safe_z(api):
        pose = dType.GetPose(api)
        move_to(api, pose[0], pose[1], CONFIG['safe_z'])
    # ② safe_z 面で XY 移動 (同座標ならスキップ)
    pose = dType.GetPose(api)
    if abs(pose[0]-x) > EPS or abs(pose[1]-y) > EPS:
        move_to(api, x, y, CONFIG['safe_z'])
    # ③ 目的高さへ降下
    move_to(api, x, y, z)

# =============== エンドエフェクタ操作 ============

def suction(api, on):
    dType.SetEndEffectorSuctionCupEx(api, 1 if on else 0, 1)


def pick_block(api):
    suction(api, True)
    time.sleep(0.25)
    # 既に safe_z ならリフト不要
    if not at_safe_z(api):
        pose = dType.GetPose(api)
        move_to(api, pose[0], pose[1], CONFIG['safe_z'])


def release_block(api):
    suction(api, False)
    time.sleep(0.25)
    # リフトは行わず、呼び出し側に任せる

# =============== 色判定 ================

def measure_color(api):
    sp = CONFIG['sensor_pos']
    move_xy_then_z(api, sp['x'], sp['y'], sp['z'])
    time.sleep(0.3)
    rgb = [dType.GetColorSensorEx(api, i) for i in range(3)]
    max_idx = rgb.index(max(rgb))
    return ['R', 'G', 'B'][max_idx]

# ============== 座標計算 ===============

def buffer_coord(color, idx):
    col   = idx // 3
    layer = idx % 3
    base  = CONFIG['buffer_base'][color]
    return (
        base['x'],
        base['y'] - col * CONFIG['buffer_interval_y'],
        base['z'] + layer * CONFIG['buffer_interval_z']
    )


def place_coord(col_idx, layer):
    base = CONFIG['place_base']
    return (
        base['x'],
        base['y'] - col_idx * CONFIG['place_interval_y'],
        base['z'] + layer * CONFIG['place_interval_z']
    )

# ============== バッファ操作 =============

def stash_in_buffer(api, color):
    idx = buffer_cnt[color]
    move_xy_then_z(api, *buffer_coord(color, idx))
    release_block(api)
    # 省リフト：次行き先確定するまで stay (Z変化無し)
    buffer_cnt[color] += 1


def retrieve_from_buffer(api, color):
    idx = buffer_cnt[color] - 1
    move_xy_then_z(api, *buffer_coord(color, idx))
    pick_block(api)
    buffer_cnt[color] -= 1

# ============== 本置き列 ================

def place_to_column(api, color, col_idx):
    layer = place_cnt[col_idx]
    move_xy_then_z(api, *place_coord(col_idx, layer))
    release_block(api)
    # リフトして安全面へ（次 XY 移動のため）
    pose = dType.GetPose(api)
    move_to(api, pose[0], pose[1], CONFIG['safe_z'])
    place_cnt[col_idx] += 1

# ============== フラッシュ試行 ================

def try_flush(api):
    for col_idx, seq in enumerate(NEXT_SEQ):
        while place_cnt[col_idx] < len(seq):
            need = seq[place_cnt[col_idx]]
            if buffer_cnt[need] == 0:
                break
            retrieve_from_buffer(api, need)
            place_to_column(api, need, col_idx)

# ============== メインループ ================

def main():
    api = dType.load()
    init_dobot(api)

    # ホーム → safe_z で待機
    h_pose = dType.GetPose(api)
    move_to(api, h_pose[0], h_pose[1], CONFIG['safe_z'])

    gp = CONFIG['grab_pos']
    wait_z = CONFIG['sensor_pos']['z']  # 常時待機する高さ(センサより上推奨)

    print("=== Sorting Loop Start ===")
    while True:
        if dType.GetInfraredSensor(api, 1)[0]:
            # ① 把持
            move_xy_then_z(api, gp['x'], gp['y'], gp['z'])
            pick_block(api)
            # ② 色測定
            color = measure_color(api)
            print(f"Detected: {color}")
            # ③ 直接本置き可能か判定
            placed = False
            for col_idx, seq in enumerate(NEXT_SEQ):
                if place_cnt[col_idx] < len(seq) and seq[place_cnt[col_idx]] == color:
                    place_to_column(api, color, col_idx)
                    placed = True
                    break
            # ④ 直接置けない→バッファ
            if not placed:
                stash_in_buffer(api, color)
            # ⑤ バッファ掃き出し
            try_flush(api)
            # ⑥ 待機位置へ
            move_xy_then_z(api, gp['x'], gp['y'], wait_z)
        time.sleep(0.05)

if __name__ == '__main__':
    main()
