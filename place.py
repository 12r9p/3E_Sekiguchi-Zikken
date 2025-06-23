import math
import time
# ───────────── 設定 (現地値を投入) ─────────────
CONFIG = {
    'grab_pos':    {'x': 252.5, 'y': 130,  'z': 14.4},
    'sensor_pos':  {'x': 191.3, 'y': 112.6,'z': 25.1},
    'buffer_base': {
        'R': {'x': 300, 'y': -65, 'z': -42},
        'G': {'x': 260, 'y': -65, 'z': -42},
        'B': {'x': 220, 'y': -65, 'z': -42},
    },
    'place_base':  {'x': 200, 'y': -150,'z': -42},
    'buffer_interval_y': 20,   # mm
    'buffer_interval_z': 15,   # mm
    'place_interval_y':  45,   # mm
    'place_interval_z':  15,   # mm
    'clearance_z':       25.1, # mm – XY 移動時の高さ
}
# ───────────────────────────────────────────────

# ランタイム状態
buffer_cnt = {'R': 0, 'G': 0, 'B': 0}
place_cnt  = [0, 0]  # 列1, 列2
NEXT_SEQ   = [['B','G','R'], ['B','R']]

# =============== Dobot 初期化 ==================

def init_dobot(api):
    dType.SetEndEffectorParamsEx(api, 59.7, 0, 0, 1)
    dType.SetColorSensor(api, 1, 2, 1)
    dType.SetInfraredSensor(api, 1, 1, 1)
    time.sleep(0.5)
    dType.SetPTPCommonParamsEx(api, 50, 50, 1)

# =============== 基本移動ユーティリティ =============

C = CONFIG  # 省略用


def move_safe(api, x, y, z):
    """常に clearance_z 面で XY 移動してから目的Zへ降下"""
    cz = C['clearance_z']
    pose = dType.GetPose(api)
    # 上昇／下降して clearance_z へ
    dType.SetPTPCmdEx(api, 0, pose[0], pose[1], cz, 0, 1)
    # XY 移動 (clearance_z 面)
    dType.SetPTPCmdEx(api, 0, x, y, cz, 0, 1)
    # 目的 Z へ降下
    dType.SetPTPCmdEx(api, 0, x, y, z, 0, 1)


def suction(api, on):
    dType.SetEndEffectorSuctionCupEx(api, 1 if on else 0, 1)


def pick_block(api):
    suction(api, True)
    time.sleep(0.2)
    # 戻り
    pose = dType.GetPose(api)
    dType.SetPTPCmdEx(api, 0, pose[0], pose[1], C['clearance_z'], 0, 1)


def release_block(api):
    suction(api, False)
    time.sleep(0.2)
    pose = dType.GetPose(api)
    dType.SetPTPCmdEx(api, 0, pose[0], pose[1], C['clearance_z'], 0, 1)

# =============== 色判定 ================

def measure_color(api):
    sp = C['sensor_pos']
    move_safe(api, sp['x'], sp['y'], sp['z'])
    time.sleep(0.15)
    rgb = [dType.GetColorSensorEx(api, i) for i in range(3)]
    return ['R','G','B'][rgb.index(max(rgb))]

# ============== 座標計算 ===============

def buffer_coord(color, idx):
    col, layer = divmod(idx, 3)
    base = C['buffer_base'][color]
    return base['x'], base['y'] - col*C['buffer_interval_y'], base['z'] + layer*C['buffer_interval_z']


def place_coord(col_idx, layer):
    base = C['place_base']
    return base['x'], base['y'] - col_idx*C['place_interval_y'], base['z'] + layer*C['place_interval_z']

# ============== バッファ / 置き列 操作 =============

def stash_buffer(api, color):
    move_safe(api, *buffer_coord(color, buffer_cnt[color]))
    release_block(api)
    buffer_cnt[color] += 1


def pull_buffer(api, color):
    move_safe(api, *buffer_coord(color, buffer_cnt[color]-1))
    pick_block(api)
    buffer_cnt[color] -= 1


def place_column(api, color, col_idx):
    move_safe(api, *place_coord(col_idx, place_cnt[col_idx]))
    release_block(api)
    place_cnt[col_idx] += 1

# ============== フラッシュ =============

def flush(api):
    for col_idx, seq in enumerate(NEXT_SEQ):
        while place_cnt[col_idx] < len(seq):
            need = seq[place_cnt[col_idx]]
            if buffer_cnt[need] == 0:
                break
            pull_buffer(api, need)
            place_column(api, need, col_idx)

# ============== メインループ =============

api = dType.load()
init_dobot(api)

# 待機状態
pose = dType.GetPose(api)
dType.SetPTPCmdEx(api, 0, pose[0], pose[1], C['clearance_z'], 0, 1)

gp = C['grab_pos']
wait_z = C['clearance_z']

print("=== Sorting Start ===")
while True:
    if dType.GetInfraredSensor(api, 1)[0]:
        # 把持
        move_safe(api, gp['x'], gp['y'], gp['z'])
        pick_block(api)
        # 色判定
        color = measure_color(api)
        print(color)
        # 直接配置 or バッファ
        placed = False
        for i, seq in enumerate(NEXT_SEQ):
            if place_cnt[i] < len(seq) and seq[place_cnt[i]] == color:
                place_column(api, color, i)
                placed = True
                break
        if not placed:
            stash_buffer(api, color)
        flush(api)
        # 戻る
        move_safe(api, gp['x'], gp['y'], wait_z)
    time.sleep(0.05)