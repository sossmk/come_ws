# 리팩터링 전(원본 come_ws) / 후 노드에 같은 입력을 넣고 발행값이 같은지 확인
# 튜닝값·분기가 그대로인지 보장하기 위한 테스트. 원본 폴더가 없으면 skip.
#   실행: source /opt/ros/jazzy/setup.bash && python3 -m pytest src/wooak/test/test_refactor_equivalence.py -q

import importlib.util
import math
import os
import random
from pathlib import Path

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
cv2 = pytest.importorskip('cv2')
from std_msgs.msg import Float32MultiArray  # noqa: E402

ORIG = Path(__file__).resolve().parents[4] / 'come_ws' / 'src' / 'wooak' / 'wooak'
pytestmark = pytest.mark.skipif(not ORIG.is_dir(), reason='원본 come_ws 없음')


def load_orig(name):
    spec = importlib.util.spec_from_file_location(f'orig_{name}', ORIG / f'{name}.py')
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def load_new(name):
    return importlib.import_module(f'wooak.{name}')


@pytest.fixture(scope='module', autouse=True)
def ros():
    os.system = lambda *a, **k: 0  # cam_exposure 의 v4l2-ctl 호출 막기
    rclpy.init()
    yield
    rclpy.shutdown()


def capture(pub):
    out = []
    pub.publish = lambda msg: out.append(msg.data)
    return out


def random_scans(n=60, seed=0):
    rnd = random.Random(seed)
    scans = []
    for _ in range(n):
        base = rnd.uniform(0.3, 3.0)
        r = [base + rnd.uniform(-0.2, 0.2) for _ in range(1860)]
        for _ in range(rnd.randint(0, 6)):   # 가까운 장애물 덩어리
            c, w, d = rnd.randrange(1860), rnd.randint(3, 120), rnd.uniform(0.15, 1.3)
            for k in range(c - w, c + w):
                r[k % 1860] = d
        for _ in range(20):                  # 노이즈
            r[rnd.randrange(1860)] = rnd.choice([float('nan'), 0.05, 17.0])
        scans.append(Float32MultiArray(data=r))
    return scans


# ---------- choice / choice1 / choice2 ----------
@pytest.mark.parametrize('name', ['choice', 'choice1', 'choice2'])
def test_choice_same_output(name):
    old = load_orig(name).LidarObstacleCheckNode()
    new_mod = load_new(name)
    captured = {}
    # 새 버전은 run(**cfg) 로 띄우므로 main 을 가로채서 설정만 꺼냄
    orig_run = new_mod.run
    new_mod.run = lambda **cfg: captured.update(cfg)
    new_mod.main()
    new_mod.run = orig_run
    new = load_new('choice_common').LidarObstacleCheckNode(**captured)
    a, b = capture(old.pub), capture(new.pub)
    for msg in random_scans():
        old.lidar_callback(msg)
        new.lidar_callback(msg)
    assert a == b and len(a) == 60
    old.destroy_node(); new.destroy_node()


# ---------- lidar_drive / maze_drive ----------
@pytest.mark.parametrize('name', ['lidar_drive', 'maze_drive'])
def test_lidar_steer_same_output(name):
    old, new = load_orig(name).LidarDriveNode(), load_new(name).LidarDriveNode()
    pairs = [(capture(old.angle_pub), capture(new.angle_pub))]
    for attr in ('finish_dir_pub', 'mission_done_pub'):
        if hasattr(old, attr):
            pairs.append((capture(getattr(old, attr)), capture(getattr(new, attr))))
    for msg in random_scans(seed=1) + [Float32MultiArray(data=[5.0] * 1860)]:
        old.lidar_callback(msg)
        new.lidar_callback(msg)
    for a, b in pairs:
        assert a == b
    assert len(pairs[0][0]) == 61
    # 조향값이 한 가지로만 나오지 않는지(테스트 입력이 분기를 충분히 타는지)
    assert len(set(pairs[0][0])) > 3
    old.destroy_node(); new.destroy_node()


# ---------- 차선 인식 ----------
def lane_images(n=40, seed=2):
    rnd = random.Random(seed)
    imgs = []
    for _ in range(n):
        img = np.full((480, 640, 3), rnd.randint(40, 90), np.uint8)
        for _ in range(rnd.randint(0, 3)):
            x_top, x_bot = rnd.randint(0, 640), rnd.randint(-100, 740)
            color = rnd.choice([(0, 220, 230), (240, 240, 240)])   # 노랑 / 흰색
            cv2.line(img, (x_top, 250), (x_bot, 420), color, rnd.randint(4, 12))
        imgs.append(img)
    return imgs


@pytest.mark.parametrize('name,delta_attr', [
    ('lane_detection_node', 'Delta_pub'),
    ('left_lane_detection_node', 'Left_Delta_pub'),
    ('right_lane_detection_node', 'Right_Delta_pub'),
])
def test_lane_delta_same_output(name, delta_attr):
    old, new = load_orig(name).LaneDetectionNode(), load_new(name).LaneDetectionNode()
    a, b = capture(getattr(old, delta_attr)), capture(getattr(new, delta_attr))
    for img in lane_images():
        old.lane_detect(img.copy())
        new.lane_detect(img.copy())
    assert a == b
    assert len(set(a)) > 3
    old.destroy_node(); new.destroy_node()


def test_calculate_slope_same():
    old = load_orig('lane_detection_node').LaneDetectionNode
    new = load_new('lane_common').calculate_slope
    rnd = random.Random(3)
    for _ in range(200):
        lines = [[rnd.randint(0, 640) for _ in range(4)] for _ in range(rnd.randint(0, 8))]
        assert old.calculate_slope(None, lines) == new(lines)


# ---------- main_drive_node 상태머신 ----------
def run_main(mod, setup, ticks=5):
    mod.time.sleep = lambda s: None
    node = mod.MainDriveNode(start_mode=setup.pop('mode'))
    cmds = []
    node.drive = lambda angle, speed: cmds.append((round(float(angle), 6), float(speed)))
    for k, v in setup.items():
        setattr(node, k, v)
    for _ in range(ticks):
        node.main_loop()
    node.destroy_node()
    return cmds


@pytest.mark.parametrize('setup', [
    {'mode': 1, 'right_delta_x': 12.0},
    {'mode': 1, 'right_delta_x': -30.0, 'stop_detected': True},
    {'mode': 1, 'obstacle_direction2': 1},
    {'mode': 2, 'right_delta_x': 7.0, 'obstacle_direction1': 1},
    {'mode': 2, 'right_delta_x': 7.0, 'obstacle_direction1': 0},
    {'mode': 3, 'delta_x': -4.0},
    {'mode': 3, 'obstacle_direction': 1},
    {'mode': 3, 'delta_x': 3.0, 'stop_detected': True, 'obstacle_flag': 1},
    {'mode': 4, 'obstacle_direction1': 1, 'lidar_angle': 1100.0},
    {'mode': 4, 'left_delta_x': 20.0, 'stop_detected': True},
    {'mode': 5, 'right_delta_x': 5.0},
    {'mode': 6, 'ticks': 1},   # 다음 틱부터는 PARKOUT(의도적으로 고친 부분)
    {'mode': 9},
])
def test_main_drive_same_commands(setup):
    setup = dict(setup)
    ticks = setup.pop('ticks', 5)
    a = run_main(load_orig('main_drive_node'), dict(setup), ticks)
    b = run_main(load_new('main_drive_node'), dict(setup), ticks)
    assert a == b and a


def test_parkout_flag_no_longer_resets():
    # 의도적으로 고친 부분: 원본은 매 틱 2초 좌회전(300)을 반복, 수정본은 한 번만
    old = run_main(load_orig('main_drive_node'), {'mode': 7, 'right_delta_x': 1.0}, ticks=4)
    new = run_main(load_new('main_drive_node'), {'mode': 7, 'right_delta_x': 1.0}, ticks=4)
    assert len([c for c in old if c[0] == 300.0]) == 4
    assert len([c for c in new if c[0] == 300.0]) == 1


def test_maze_uses_maze_steer():
    # 의도적으로 고친 부분: MAZE 모드는 /maze_steer 값을 씀
    new = run_main(load_new('main_drive_node'),
                   {'mode': 8, 'obstacle_direction1': 1, 'lidar_angle': 111.0, 'maze_steer': 999.0}, ticks=1)
    assert new == [(999.0, 200.0)]


def test_no_nan_leaks():
    # valid_ranges 가 NaN 을 20.0 으로 바꾸는지
    vr = load_new('lidar_common').valid_ranges
    assert vr([float('nan'), 0.05, 17.0, 1.0]) == [20.0, 20.0, 20.0, 1.0]
    assert not any(math.isnan(x) for x in vr([float('nan')] * 5))
