import DobotDLL as dType
import time

# ────────────── 設定（後で外部から注入可能） ──────────────
CONFIG = {
    # 座標は現地合わせでセットしてください
    'grab_pos':    {'x': None, 'y': None, 'z': None},   # ブロック把持位置
    'sensor_pos':  {'x': None, 'y': None, 'z': None},   # カラーセンサ計測位置
    'place_base':  {'x': None, 'y': None, 'z': None},   # ブロック設置列開始位置
    'interval_y':  None,   # Y方向間隔
    'interval_z':  None,   # Z方向積み重ね間隔
    'safe_z':      None,   # X,Y移動時の安全高度
}
# ────────────────────────────────────────────────────────

# 各色カウント
counters = {'R': 0, 'G': 0, 'B': 0}

def init_dobot(api):
    """Dobot の初期化設定を行う"""
    dType.SetEndEffectorParamsEx(api, 59.7, 0, 0, 1)
    dType.SetColorSensor(api, 1, 2, 1)
    dType.SetInfraredSensor(api, 1, 1, 1)
    time.sleep(1)
    dType.SetPTPJointParamsEx(api,600,600,600,600,600,600,600,600,1)
    dType.SetPTPCommonParamsEx(api,50,50,1)
    dType.SetPTPJumpParamsEx(api,50,100,1)


def move_to(api, x, y, z, r=0):
    """ワールド座標へ PTP 移動"""
    dType.SetPTPCmdEx(api, 0, x, y, z, r, 1)


def move_xy_then_z(api, x, y, z):
    """先に X,Y を安全高度へ移動し、そこから Z を降ろす"""
    sz = CONFIG['safe_z']
    move_to(api, x, y, sz)
    move_to(api, x, y, z)


def pick_block(api):
    """吸着 ON → 安全高度へリフト"""
    dType.SetEndEffectorSuctionCupEx(api, 1, 1)
    time.sleep(0.5)
    # 吸着後は安全高度に戻す
    # 現在の X,Y 座標を取得
    pose = dType.GetPose(api)
    move_to(api, pose[0], pose[1], CONFIG['safe_z'])


def release_block(api):
    """吸着 OFF → 安全高度へリフト"""
    dType.SetEndEffectorSuctionCupEx(api, 0, 1)
    time.sleep(0.5)
    pose = dType.GetPose(api)
    move_to(api, pose[0], pose[1], CONFIG['safe_z'])


def get_color(api):
    """色センサから色を読み取り、'R'/'G'/'B' を返す"""
    sp = CONFIG['sensor_pos']
    # 安全高度経由でセンサ上空へ
    move_xy_then_z(api, sp['x'], sp['y'], sp['z'])
    time.sleep(1)
    r = dType.GetColorSensorEx(api, 0)
    g = dType.GetColorSensorEx(api, 1)
    b = dType.GetColorSensorEx(api, 2)
    mx = max(r, g, b)
    return 'R' if mx == r else 'G' if mx == g else 'B'


def place_block(api, color):
    """指定色の列に順次積み上げる"""
    pb = CONFIG['place_base']
    iy = CONFIG['interval_y']
    iz = CONFIG['interval_z']
    idx = counters[color]
    # Y方向オフセットマッピング
    offset_y = {
        'R': +1 * iy,
        'G':  0 * iy,
        'B': -1 * iy
    }[color]
    x = pb['x']
    y = pb['y'] + offset_y
    z = pb['z'] + idx * iz
    # 真上まで移動してから下降
    move_xy_then_z(api, x, y, z)
    release_block(api)
    counters[color] += 1


def main():
    api = dType.load()
    init_dobot(api)

    # ホーム位置復帰
    home = dType.GetPose(api)
    move_to(api, home[0], home[1], CONFIG['safe_z'])

    print("=== Sorting Loop Start ===")
    while True:
        # フトロセンサーでブロック検出
        if dType.GetInfraredSensor(api, 1)[0] == 1:
            # ブロック把持: 安全高度経由で下降
            gp = CONFIG['grab_pos']
            move_xy_then_z(api, gp['x'], gp['y'], gp['z'])
            pick_block(api)

            # 色取得 → 配置
            color = get_color(api)
            print(f"Detected: {color}")
            place_block(api, color)

            # 次の検出待ち位置へリフト
            move_to(api, gp['x'], gp['y'], CONFIG['sensor_pos']['z'])
        time.sleep(0.1)


if __name__ == '__main__':
    main()
