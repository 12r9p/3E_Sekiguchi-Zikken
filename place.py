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
    'place_interval_y': 45,
    # 本置き積層間隔 (Z方向)
    'place_interval_z': 15,
    # XY移動時の安全高度
    'clearance_z':       50,
}
# ────────────────────────────────────────────────────────

# ランタイム状態
buffer_cnt  = {'R': 0, 'G': 0, 'B': 0}
place_cnt   = [0, 0]
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

# =============== ユーティリティ =============
EPS = 0.1


def _clearance():
    # clearance_z 未設定なら sensor_z を既定値とする
    return CONFIG['clearance_z'] if CONFIG['clearance_z'] is not None else CONFIG['sensor_pos']['z']


def at_clearance(api):
    return abs(dType.GetPose(api)[2] - _clearance()) < EPS


def move_to(api, x, y, z, r=0):
    dType.SetPTPCmdEx(api, 0, x, y, z, r, 1)


def move_xy_then_z(api, x, y, z):
    cz = _clearance()
    pose = dType.GetPose(api)
    # 1) 必要ならその場で Z↑clearance
    if pose[2] + EPS < cz:
        move_to(api, pose[0], pose[1], cz)
    # 2) XY 移動 (同座標ならスキップ)
    if abs(pose[0]-x) > EPS or abs(pose[1]-y) > EPS:
        move_to(api, x, y, cz)
    # 3) 降下
    move_to(api, x, y, z)

# =============== エンドエフェクタ操作 ============

def suction(api, on):
    dType.SetEndEffectorSuctionCupEx(api, 1 if on else 0, 1)


def pick_block(api):
    suction(api, True)
    time.sleep(0.2)
    # 戻りは clearance_z のみ
    if not at_clearance(api):
        pose = dType.GetPose(api)
        move_to(api, pose[0], pose[1], _clearance())


def release_block(api, lift_after=False):
    suction(api, False)
    time.sleep(0.2)
    if lift_after and not at_clearance(api):
        pose = dType.GetPose(api)
        move_to(api, pose[0], pose[1], _clearance())

# =============== 色判定 ================

def measure_color(api):
    sp = CONFIG['sensor_pos']
    move_xy_then_z(api, sp['x'], sp['y'], sp['z'])
    time.sleep(0.25)
    rgb = [dType.GetColorSensorEx(api, i) for i in range(3)]
    return ['R','G','B'][rgb.index(max(rgb))]

# ============== 座標計算 ===============

def buffer_coord(color, idx):
    col, layer = divmod(idx, 3)
    base = CONFIG['buffer_base'][color]
    return base['x'], base['y'] - col*CONFIG['buffer_interval_y'], base['z'] + layer*CONFIG['buffer_interval_z']


def place_coord(col_idx, layer):
    base = CONFIG['place_base']
    return base['x'], base['y'] - col_idx*CONFIG['place_interval_y'], base['z'] + layer*CONFIG['place_interval_z']

# ============== バッファ操作 =============

def stash_in_buffer(api, color):
    move_xy_then_z(api, *buffer_coord(color, buffer_cnt[color]))
    release_block(api)  # lift_after=False なのでその場待機
    buffer_cnt[color] += 1


def retrieve_from_buffer(api, color):
    move_xy_then_z(api, *buffer_coord(color, buffer_cnt[color]-1))
    pick_block(api)
    buffer_cnt[color] -= 1

# ============== 本置き列 ================

def place_to_column(api, color, col_idx):
    move_xy_then_z(api, *place_coord(col_idx, place_cnt[col_idx]))
    release_block(api, lift_after=True)  # 次の動作のために clearance_z へ戻す
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

api = dType.load()
init_dobot(api)

# 初期待機は clearance_z で
pose = dType.GetPose(api)
move_to(api, pose[0], pose[1], _clearance())

gp = CONFIG['grab_pos']
wait_z = _clearance()

print("=== Sorting Loop Start ===")
while True:
    if dType.GetInfraredSensor(api, 1)[0]:
        # 把持
        move_xy_then_z(api, gp['x'], gp['y'], gp['z'])
        pick_block(api)
        # 色計測
        color = measure_color(api)
        print(f"Detected: {color}")
        # 直接置ける？
        placed = False
        for col_idx, seq in enumerate(NEXT_SEQ):
            if place_cnt[col_idx] < len(seq) and seq[place_cnt[col_idx]] == color:
                place_to_column(api, color, col_idx)
                placed = True
                break
        if not placed:
            stash_in_buffer(api, color)
        try_flush(api)
        # 待機へ
        move_xy_then_z(api, gp['x'], gp['y'], wait_z)
    time.sleep(0.05)
