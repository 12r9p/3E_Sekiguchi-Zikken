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
        'G': {'x': 260, 'y': -65, 'z': 42},
        'B': {'x': 220, 'y': -65, 'z': 42},
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

# カウンタ
buffer_counters = {'R': 0, 'G': 0, 'B': 0}
place_counters  = [0, 0]   # 列1, 列2

# ターゲットシーケンス（青= 'B', 緑='G', 赤='R'）
NEXT_SEQ = [['B','G','R'], ['B','R']]

# Dobot初期化
def init_dobot(api):
    dType.SetEndEffectorParamsEx(api, 59.7, 0, 0, 1)
    dType.SetColorSensor(api, 1, 2, 1)
    dType.SetInfraredSensor(api, 1, 1, 1)
    time.sleep(1)
    dType.SetPTPJointParamsEx(api,600,600,600,600,600,600,600,600,1)
    dType.SetPTPCommonParamsEx(api,50,50,1)
    dType.SetPTPJumpParamsEx(api,50,100,1)

# 移動
def move_to(api, x, y, z, r=0):
    dType.SetPTPCmdEx(api, 0, x, y, z, r, 1)

def move_xy_then_z(api, x, y, z):
    sz = CONFIG['safe_z']
    move_to(api, x, y, sz)
    move_to(api, x, y, z)

# 吸着・放棄
def pick_block(api):
    dType.SetEndEffectorSuctionCupEx(api, 1, 1)
    time.sleep(0.3)
    # 吸着後リフト
    pose = dType.GetPose(api)
    move_to(api, pose[0], pose[1], CONFIG['safe_z'])

def release_block(api):
    dType.SetEndEffectorSuctionCupEx(api, 0, 1)
    time.sleep(0.3)
    pose = dType.GetPose(api)
    move_to(api, pose[0], pose[1], CONFIG['safe_z'])

# 色取得
def get_color(api):
    sp = CONFIG['sensor_pos']
    move_xy_then_z(api, sp['x'], sp['y'], sp['z'])
    time.sleep(0.5)
    vals = [dType.GetColorSensorEx(api, i) for i in range(3)]
    mx = max(vals)
    return ['R','G','B'][vals.index(mx)]

# バッファからピック
def pick_from_buffer(api, color):
    idx = buffer_counters[color] - 1
    col = idx // 3
    layer = idx % 3
    base = CONFIG['buffer_base'][color]
    x = base['x']
    y = base['y'] - col * CONFIG['buffer_interval_y']
    z = base['z'] + layer * CONFIG['buffer_interval_z']
    move_xy_then_z(api, x, y, z)
    pick_block(api)
    buffer_counters[color] -= 1

# 本置き + リリース
def place_in_column(api, color, col_idx):
    idx  = place_counters[col_idx]
    base = CONFIG['place_base']
    x = base['x']
    y = base['y'] - col_idx * CONFIG['place_interval_y']
    z = base['z'] + idx * CONFIG['place_interval_z']
    move_xy_then_z(api, x, y, z)
    release_block(api)
    place_counters[col_idx] += 1

# バッファ for まだ組立できないもの
def place_in_buffer(api, color):
    idx = buffer_counters[color]
    col = idx // 3
    layer = idx % 3
    base = CONFIG['buffer_base'][color]
    x = base['x']
    y = base['y'] - col * CONFIG['buffer_interval_y']
    z = base['z'] + layer * CONFIG['buffer_interval_z']
    move_xy_then_z(api, x, y, z)
    release_block(api)
    buffer_counters[color] += 1

# バッファ内から必要な色を組立列へ
def try_flush(api):
    for i, seq in enumerate(NEXT_SEQ):
        while place_counters[i] < len(seq):
            need = seq[place_counters[i]]
            if buffer_counters[need] <= 0:
                break
            pick_from_buffer(api, need)
            place_in_column(api, need, i)


api = dType.load()
init_dobot(api)
# ホーム → 安全高度
home = dType.GetPose(api)
move_to(api, home[0], home[1], CONFIG['safe_z'])

print("=== Sorting Loop Start ===")
while True:
    if dType.GetInfraredSensor(api, 1)[0] == 1:
        gp = CONFIG['grab_pos']
        move_xy_then_z(api, gp['x'], gp['y'], gp['z'])
        pick_block(api)
        color = get_color(api)
        print(f'Detected: {color}')
        # 直接配置可能なら本置き, それ以外はバッファ
        placed = False
        for i, seq in enumerate(NEXT_SEQ):
            if place_counters[i] < len(seq) and seq[place_counters[i]] == color:
                place_in_column(api, color, i)
                placed = True
                break
        if not placed:
            place_in_buffer(api, color)
        try_flush(api)
        # 待機位置へリフト
        move_to(api, gp['x'], gp['y'], CONFIG['sensor_pos']['z'])
    time.sleep(0.1)
