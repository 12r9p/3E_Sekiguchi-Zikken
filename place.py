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

# 初期化
def init_dobot(api):
    dType.SetEndEffectorParamsEx(api, 59.7, 0, 0, 1)
    dType.SetColorSensor(api, 1, 2, 1)
    dType.SetInfraredSensor(api, 1, 1, 1)
    time.sleep(1)
    dType.SetPTPJointParamsEx(api,600,600,600,600,600,600,600,600,1)
    dType.SetPTPCommonParamsEx(api,50,50,1)
    dType.SetPTPJumpParamsEx(api,50,100,1)

# 基本移動関数
def move_to(api, x, y, z, r=0):
    dType.SetPTPCmdEx(api, 0, x, y, z, r, 1)

def move_xy_then_z(api, x, y, z):
    # 1) XYを安全高度で移動 → 2) 目的Zへ降下
    safe_z = CONFIG['safe_z']
    move_to(api, x, y, safe_z)
    move_to(api, x, y, z)

# 吸着／放棄
def pick_block(api):
    dType.SetEndEffectorSuctionCupEx(api, 1, 1)
    time.sleep(0.5)
    # 吸着後リフト
    pose = dType.GetPose(api)
    move_to(api, pose[0], pose[1], CONFIG['safe_z'])

def release_block(api):
    dType.SetEndEffectorSuctionCupEx(api, 0, 1)
    time.sleep(0.5)
    # 放棄後リフト
    pose = dType.GetPose(api)
    move_to(api, pose[0], pose[1], CONFIG['safe_z'])

# 色取得
def get_color(api):
    sp = CONFIG['sensor_pos']
    move_xy_then_z(api, sp['x'], sp['y'], sp['z'])
    time.sleep(1)
    r = dType.GetColorSensorEx(api, 0)
    g = dType.GetColorSensorEx(api, 1)
    b = dType.GetColorSensorEx(api, 2)
    mx = max(r, g, b)
    return 'R' if mx == r else 'G' if mx == g else 'B'

# ────────── バッファ配置 ──────────
def place_in_buffer(api, color):
    """
    各色バッファに front-right 起点から格納。
    - 3個で1列 -> 次の列へ
    - 列方向は -Y 方向
    - 積層は +Z 方向
    """
    idx   = buffer_counters[color]
    col   = idx // 3
    layer = idx % 3
    base  = CONFIG['buffer_base'][color]
    x = base['x']
    y = base['y'] - col * CONFIG['buffer_interval_y']
    z = base['z'] + layer * CONFIG['buffer_interval_z']
    move_xy_then_z(api, x, y, z)
    release_block(api)
    buffer_counters[color] += 1

# ────────── 本置き列配置 ──────────
# 2列分 帰無 as needed
NEXT_SEQ = [['R','G','B'], ['R','B']]
def try_flush(api):
    for i, seq in enumerate(NEXT_SEQ):
        while place_counters[i] < len(seq):
            need = seq[place_counters[i]]
            if buffer_counters[need] == 0:
                break
            # バッファから取り出し
            buffer_counters[need] -= 1
            place_in_column(api, need, i)


def place_in_column(api, color, col_idx):
    """
    front-right 起点から 2列並べて積み上げ。
    - 列方向は -Y 方向
    - 積層は +Z 方向
    """
    idx  = place_counters[col_idx]
    base = CONFIG['place_base']
    x = base['x']
    y = base['y'] - col_idx * CONFIG['place_interval_y']
    z = base['z'] + idx * CONFIG['place_interval_z']
    move_xy_then_z(api, x, y, z)
    release_block(api)
    place_counters[col_idx] += 1

# メインループ
def main():
    api = dType.load()
    init_dobot(api)
    # 初期リフト
    home = dType.GetPose(api)
    move_to(api, home[0], home[1], CONFIG['safe_z'])

    print("=== Sorting Loop Start ===")
    while True:
        if dType.GetInfraredSensor(api, 1)[0] == 1:
            # 把持位置へ下降
            gp = CONFIG['grab_pos']
            move_xy_then_z(api, gp['x'], gp['y'], gp['z'])
            pick_block(api)
            color = get_color(api)
            print(f"Detected: {color}")
            place_in_buffer(api, color)
            try_flush(api)
            # 次検出準備
            move_to(api, gp['x'], gp['y'], CONFIG['sensor_pos']['z'])
        time.sleep(0.1)

if __name__ == '__main__':
    main()
