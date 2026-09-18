# come_ws 리팩터링 보고서

> 대상: `IRC_BACKUP_26/come_ws` (ROS2 Jazzy, `wooak` 패키지 — 카메라 차선/정지선 + YDLIDAR G6 + Dynamixel 자율주행)
> 결과물: `IRC_BACKUP_26/come_ws_refactored/` (브랜치 `refactor/cleanup`, 원본 `come_ws` 는 변경 0건)
> 원칙: **원문 느낌 유지** — 파일명·토픽명·튜닝값·분기 구조·한글 주석·이모지 로그는 그대로. 확실한 버그만 고치고, 주행이 달라질 수 있는 것은 보고만.

---

## 1. 요약

| 항목 | 결과 |
|---|---|
| 확실한 버그 수정 | **12건** (실행 불가 3, 잘못된 경로/엔트리 4, 동작 버그 2, 기타 3) |
| 중복 제거 | 공통 모듈 4개 신설(`lane_common`, `lidar_common`, `dxl_common`, `choice_common`) |
| 활성 코드 줄 수 | `wooak/wooak/*.py` **2206 → 1847줄** (공통 모듈 265줄 포함) |
| 레거시 정리 | 24개 파일을 `src/wooak/legacy/`(COLCON_IGNORE)로 이동 + 이유 README |
| 보고만 한 항목 | 11건 (트랙 재튜닝 필요) |
| 검증 | py_compile 전체 통과 · `colcon build` 성공 · 실행파일 14개 등록 · **동등성 테스트 25/25 통과** · 노드 기동 스모크 테스트 통과 |

커밋 내역 (`git log main..refactor/cleanup`):
1. `legacy/ 로 미사용·중복 파일 이동, 깨진 주석 인코딩 복원`
2. `라이다/다이나믹셀 공통 모듈 분리, choice* 통합, 확실한 버그 수정`
3. `차선 인식 공통화(lane_common), main_drive/stopline 정리, setup.py·launch 수정, 동등성 테스트`

---

## 2. 확실한 버그 수정

| # | 위치(원본) | 문제 | 수정 | 주행 영향 |
|---|---|---|---|---|
| 1 | `motor_control_node.py:9` | `import Float32` → **ImportError, 노드 자체가 안 뜸** | `from std_msgs.msg import Float32` | 없음(원래 실행 불가) |
| 2 | `motor_control_node.py:146` | `Float32(data=steer_pos)` 에 int 전달 → rclpy AssertionError | `float(steer_pos)` | 없음 |
| 3 | `launch/bringup_all.launch.py:27` | 3칸 들여쓰기 **IndentationError** | 들여쓰기 수정 (레거시로 이동) | 없음 |
| 4 | `setup.py` | `lidar_array.py = wooak.lidar_array:main` — **없는 모듈** / `scan_to_cmdvel = scan_to_cmdvel:main` — 패키지 밖 | 실제 모듈로 연결 | 없음 |
| 5 | `setup.py` | `lidar_array_node` 가 레거시 1440bin 버전을 가리킴 → 하위 노드들이 `n<1800` 으로 전부 무시 | `wooak.lidar_array_node`(1860bin)로 | `bringup.launch` 사용 시 라이다 주행이 실제로 동작하게 됨 |
| 6 | `setup.py` | 차선 3종·choice 3종·motor_control 미등록 → launch 가 절대경로로 우회 | 전부 `console_scripts` 등록 (기존 이름 유지) | 없음 |
| 7 | `launch/camall.launch.py` | `ExecuteProcess(['python3', '/home/kkk/come_ws/...'])` — **다른 PC 에서 실행 불가** | `Node(package='wooak', executable=...)` | 없음 |
| 8 | `launch/bringup.launch.py` | `executable='lidar_drive'` (없음, 실제는 `lidar_drive.py`) | 이름 수정 | 없음 |
| 9 | `main_drive_node.py` PARKOUT | `self.parkout_flag=0` 이 **매 틱 실행** → 2초 좌회전(300)을 무한 반복, 정지선 카운트 진행 불가 | 초기화를 `__init__` 한 번으로 | **의도적 변경** |
| 10 | `lidar_drive.py` ↔ `maze_drive.py` | 둘 다 `/lidar_steer` 로 발행 (`lidarall.launch` 가 둘 다 띄움) → 두 노드 조향값이 섞임. main 은 `/maze_steer` 를 구독하지만 발행자 없음 | maze_drive → `/maze_steer`, MAZE 모드는 `self.maze_steer` 사용, 노드명 `maze_drive_node` | **의도적 변경** |
| 11 | `choice.py/1/2` | 세 노드 이름이 모두 `lidar_obstacle_check_node` → 동시 실행 시 이름 충돌 | `…node`, `…node1`, `…node2` | 없음 |
| 12 | 모든 노드 `main()` | Jazzy 에서 Ctrl+C 시 `rclpy.shutdown()` 이 **RCLError 트레이스백** (이미 shutdown 됨) | `rclpy.try_shutdown()` | 없음 |

추가로 `main_drive_node.py`(3줄), `dyna_cmd_vel_node.py`(30여 줄)의 **깨진 한글 주석(UTF-8 → cp1252 모지바케)** 을 복원했습니다. 저장 과정에서 `0xA0` 바이트가 공백으로 바뀐 것까지 역추적해 원문을 되살렸습니다.
예) `#ì¢Œ -> ë¨¸ë¦¬ ì •ë ¬` → `#좌 -> 머리 정렬`, `#ìž¥ì• ë¬¼ ì•ˆë§Œë‚¬ì„ë•Œëž‘…` → `#장애물 안만났을때랑 장애물 통과 후 양선차선 주행`

---

## 3. 중복·구조 개선

### 3-1. 공통 모듈

| 신규 모듈 | 흡수한 중복 | 사용처 |
|---|---|---|
| `lane_common.py` (133줄) | `cam_exposure()` ×3, `calculate_slope()` ×3, 해상도 자동감지 `image_callback` ×3, `visualize_lanes`/`publish_result_image` ×3 → `LaneNodeBase(Node)` | 차선 노드 3종 |
| `lidar_common.py` (40줄) | 거리 유효성 리스트컴프리헨션 ×7, 구역 최소값/argmin ×5, `1800<n<1900` 검사, 조향 클램프, `STEER_STEP` | lidar_drive, maze_drive, motor_control, choice_common, lidar_array |
| `dxl_common.py` (31줄) | 다이나믹셀 제어테이블·ID(7/8/9)·모드·`STEER_CENTER/MIN/MAX` ×2 | m2ct, motor_control, (lidar_common 경유) |
| `choice_common.py` (61줄) | `choice.py`·`choice1.py`·`choice2.py` — **4개 상수만 다른 63줄 복붙 3개** | choice* (각 19줄, 상수만) |

### 3-2. 파일별 줄 수 (원본 → 리팩터링)

| 파일 | 원본 | 이후 | 비고 |
|---|---:|---:|---|
| choice / choice1 / choice2 | 63 ×3 | 19 ×3 | 설정값만 남김. 미사용 `right_min`(1300개 min, 매 프레임) 제거 |
| lane_detection_node | 288 | 161 | `gap = (abs(x2-x1) or abs(x1-x2))` → `abs(x2-x1)` |
| left_lane_detection_node | 268 | 118 | 결과에 안 쓰이던 `adaptiveThreshold` 계산 제거, 주석 처리된 죽은 코드 정리 |
| right_lane_detection_node | 243 | 126 | 〃 |
| lidar_drive / maze_drive | 133 / 133 | 126 / 127 | 미사용 `v_left/v_right` 제거 |
| motor_control_node | 177 | 161 | |
| m2ct | 124 | 106 | 미사용 `angle_gain` 제거 |
| main_drive_node | 364 | 368 | 아래 참고 (주석 증가) |
| stopline_find | 189 | 154 | 존재하지 않는 `publish_result_image` 를 부르던 죽은 `visualize_lanes` 제거, 미사용 import 정리 |

### 3-3. main_drive_node 정리 (분기 구조는 그대로)
- `800.0 + self.steer_gain * x` 11곳 → `self.lane_steer(x)` (`STEER_CENTER` 상수)
- `hasattr(self, 'second_drive_start')` 식 첫 진입 판정 → `__init__` 에서 `None` 초기화
- 50Hz 타이머에서 **매 틱 찍던 모드 로그 35곳** → `throttle_duration_sec=0.5`
- 미사용 import(`String`, `Float32MultiArray`, `math`) 제거

### 3-4. 패키지/빌드
- `package.xml`: 중복 `rclpy` 제거, 누락 의존성 `python3-opencv`, `python3-numpy`, `usb_cam`, `ydlidar_ros2_driver` 추가
- `setup.py`: `find_packages(exclude=[..., 'legacy'])`

---

## 4. 레거시 이동 (`src/wooak/legacy/`)

| 항목 | 이유 |
|---|---|
| `dyna_lidar_bringup/` (8파일) | RPLIDAR 시절 코드. `package='dyna_lidar_bringup'` 은 존재한 적 없는 패키지. `lidar_array_node.py` 는 `scan_callback` 이 **두 번 정의**(앞쪽은 죽은 코드), 1440bin 이라 G6 노드들과 비호환 |
| `scripts/motor.py`, `scan_to_cmdvel.py` | `odom_from_dxl.py` 와 중복 / 엔트리 경로 오류로 실행 불가 |
| `launch/{bringup_all, cmdvel_test, lidar_gap_steer, odom_test}` | 없는 패키지·실행파일(`lidar_gap_steer`) 참조, `/home/kangsanmaru/...` 하드코딩 |
| `config/lidar_gap_steer.launch.py` | launch 쪽과 거의 동일한 사본 |
| `wooak/wooak/launch/` | `launch/` 의 사본(설치도 안 됨) |
| `yolov8_train.py`, `yolov8_project/` | Windows 경로(`C:/home/...`)의 데스크톱 학습 스크립트 |
| `wooak_backup.zip`, `package.xml.bak` | 예전 백업 |

---

## 5. 보고만 한 항목 (트랙에서 재튜닝이 필요해 코드 유지)

우선순위 순.

| 우선 | 위치 | 내용 | 권장 |
|---|---|---|---|
| **높음** | `main_drive_node.main_loop` | 타이머 콜백 안의 `time.sleep()` (OBSTACLE 최대 **13.7초**, PARKING 8초…) 동안 **모든 구독 콜백이 멈춤** → sleep 직후 `stop_detected` 등은 옛날 값 | 모드별 "시작 시각 + 경과시간" 비차단 시퀀스로 전환 (주행 타이밍 재확인 필요) |
| **높음** | PARKOUT flag1→2→3 | 한 틱 안에서 `stop_detected` 를 리셋하지 않아, **정지선 하나에 flag 1·2·3 이 연쇄로 통과**해 바로 MAZE 로 감 (의도는 정지선 3개 세기로 보임) | 각 전이마다 `self.stop_detected=False` + 한 틱에 한 단계(`elif`) |
| **높음** | MAZE / FINISH | `/maze_done`, `/cone_done` 은 구독만 하고 안 씀 → **FINISH 모드에 도달할 경로가 없음**. 게다가 `lidar_drive` 는 `/cone_done` 퍼블리셔만 만들고 한 번도 발행하지 않음. `/lidar_speed` 는 발행자 없음, `/finish_dir` 은 구독자 없음 | 완료 신호로 모드 전환 연결 |
| 중간 | Second_drive | `fixed_direction` 0/1 분기가 **둘 다 `right_delta_x`** 사용 (로그는 "Left lane 유지 중") | 트랙 영상으로 의도 확인 |
| 중간 | 차선 노드 3종 | 각자 시작 시 `v4l2-ctl` 을 호출 → **마지막에 뜬 노드 설정이 이김**. right 만 `auto_exposure=3`(자동). launch 의 `usb_cam` 파라미터(exposure 60)와도 중복 | 노출은 `usb_cam` 파라미터 한 곳에서만 |
| 중간 | `motor_control_node` ↔ `m2ct` | 같은 다이나믹셀 버스에 **동시에 명령**을 씀 (둘 다 띄우면 충돌). motor_control 은 포트 열기 실패 시 `return` 후 `destroy_node` 에서 닫힌 포트에 씀 | motor_control 은 테스트용으로만 |
| 중간 | `main()` | `input("Enter start mode")` → launch 로 띄우면 입력 대기로 멈춤 | ROS 파라미터 `start_mode` (기본 1) |
| 낮음 | `lane_detection_node` | `View_Center = (roi_start_x + roi_center_width)//2` — roi_start_x=0 일 때만 화면 중앙 | 해상도 바뀌면 재확인 |
| 낮음 | `stopline_find` | `neighbors = row_counts[best_idx:best_idx+1]` → `avg_coverage == coverage` (사실상 조건 하나). 초기 ROI 0.7 vs 해상도 변경 시 0.6 | 의도 확인 |
| 낮음 | PARKIN | `elapsed<5.0` / `5.0<elapsed<15.0` — 정확히 5.0 이면 else(왼쪽 차선)로 빠짐 | `<=` |
| 낮음 | `lidar_drive` | `state` 문자열을 만들지만 로그를 안 찍음(pyflakes 경고) | 디버깅 필요 시 로그 추가 |

`yolo_ros`(서드파티, 별도 git)는 손대지 않았습니다. 참고로 `yolo_sign.py` 는 setup.py 에 등록되지 않았고 클래스명이 `yolo_node.py` 와 같은 `YoloNode` 이며, `traffic_light_color_detector.py` 는 `print()` 디버그 출력이 매 프레임 발생합니다.

디버그 오버레이 차이 1건: `right_lane_detection_node` 의 결과 이미지 라벨 `L:{x_right_top}` → `R:{x_right_top}` (발행되는 delta 값은 동일).

---

## 6. 검증

```bash
cd come_ws_refactored && source /opt/ros/jazzy/setup.bash
colcon build --packages-select wooak && source install/setup.bash
python3 -m pytest src/wooak/test/test_refactor_equivalence.py -q   # 25 passed
```

| 검증 | 결과 |
|---|---|
| `py_compile` (legacy 포함 전 파일) | 통과 |
| `colcon build --packages-select wooak` | 성공 |
| `ros2 pkg executables wooak` | 14개 (원본은 13개 중 5개가 없는 모듈/경로) |
| **동등성 테스트** `test_refactor_equivalence.py` | **25/25 통과** — 원본(`../come_ws`)과 리팩터링 노드에 같은 입력을 넣고 발행값 비교: choice×3(스캔 60개), lidar_drive/maze_drive(61개, 조향값 다양성 확인), 차선 노드×3(합성 이미지 40장), `calculate_slope` 200회, main_drive 모드 1~6·9 명령열 13케이스, 의도적 변경 2건(PARKOUT·MAZE)은 **새 동작**을 검증 |
| 노드 기동 스모크 (`timeout 3 ros2 run`) | 카메라·라이다·main 노드 전부 정상 기동/종료, Ctrl+C 트레이스백 0건 |
| 모터 노드 | 이 PC 에 `dynamixel_sdk` 가 없어 스텁 SDK 로 확인: motor_control 조향 300 + 속도 쓰기, m2ct 1500→1300 클램프·좌우 반전 정상 |
| 실차 | **미검증** — 하드웨어 필요. 의도적 변경 2건(PARKOUT, MAZE `/maze_steer`)은 트랙에서 확인 필요 |

---

## 7. understand-anything 분석

`/understand` 로 `come_ws_refactored` 를 분석했습니다. 분석 대상은 실제로 쓰는 코드 30개 파일이고, legacy/·yolo_ros·빌드 산출물은 `.ua/.understandignore` 로 제외했습니다. 결과는 `.ua/knowledge-graph.json` 에 있으며, 대시보드는 `/understand-dashboard` 로 열 수 있습니다.

| 항목 | 값 |
|---|---|
| 노드 | 80개: file 26, function 38, class 12, config 4 |
| 엣지 | 200개: contains 50, exports 39, calls 34, depends_on 34, imports 17, related 17(ROS 토픽 연결), tested_by 4, inherits 3, configures 2 |
| 레이어 | 6개 |
| 가이드 투어 | 13단계 |
| 검증 | issue 0건, 경고 1건(빈 `__init__.py` 는 연결이 없음, 정상) |

### 7-1. 레이어 구조 (ROS 데이터 흐름 순서)

```
 [인식: 카메라 5]            [인식·주행: 라이다 8]
  lane_common ◀─ lane×3        lidar_array_node (/scan → /lidar_array)
  stopline_find                lidar_common ◀─ choice_common ◀─ choice×3
        │ Delta_x, stop_line    lidar_drive (/lidar_steer), maze_drive (/maze_steer)
        ▼                               │ obstacle_direction*, steer
     [주행 결정: 상태머신 1] main_drive_node ◀┘
        │ /motor_command/angle, speed
        ▼
     [구동: 모터 3]  m2ct ─▶ dxl_common ◀─ motor_control_node(구버전 독립 노드)
 [실행·빌드 설정 9]  launch×3, setup.py, package.xml, setup.cfg, usb_cam yaml …
 [테스트 4]          test_refactor_equivalence + ament lint 3종
```

### 7-2. 변경 영향 분석 (그래프 기준)

| 모듈 | import fan-in | 의미 |
|---|---:|---|
| `lidar_common` | **8** | 라이다 쪽 노드가 모두 여기에 의존합니다. `valid_ranges`/구역 인덱스를 고치면 choice×3, lidar_drive, maze_drive, motor_control, lidar_array 가 함께 바뀌므로 반드시 `test_refactor_equivalence` 로 확인해야 합니다 |
| `lane_common` | 3 | 차선 노드 3종의 `LaneNodeBase` 가 3개 노드에 inherits 로 연결됩니다. ROI 비율이나 해상도 처리 변경은 세 노드 모두에 영향을 줍니다 |
| `dxl_common` | 3 | `STEER_CENTER/MIN/MAX` 는 단일 출처이며 lidar_common 을 통해 조향 클램프에도 쓰입니다 |
| `choice_common` | 3 | choice×3 에 대응합니다 |
| `main_drive_node` | 0 (related 11) | import 로 연결되지 않고 **ROS 토픽으로만** 결합된 허브입니다. 토픽 이름을 바꾸면 그래프가 잡아내지 못하므로 토픽 이름은 가능한 한 그대로 두었습니다 |

- 원본은 중복 코드끼리 연결이 없는 **고립 파일들**이었습니다(원본 `importMap` 기준 wooak 내부 import 0개). 리팩터링 후에는 import 17개로 공통 모듈에 모이는 구조가 되어, 한 곳을 고치면 모든 사용처에 반영됩니다.
- 위험 지점: `lidar_common` 이 가장 많이 참조되는 모듈이라 변경 위험이 가장 큽니다. 동등성 테스트가 이 모듈의 결과를 원본과 직접 비교하므로 안전망 역할을 합니다.

### 7-3. 가이드 투어 (대시보드에서 순서대로)

1. 실행 구성: launch 파일 → 2. 중앙 상태머신 → 3. 공통 차선 모듈 → 4. 차선 인식 노드 3종 → 5. 정지선 검출 → 6. 라이다 전처리 → 7. 공통 라이다 모듈 → 8. 정면 장애물 판별 → 9. 라바콘·미로 주행 → 10. 모터 구동 → 11. 구버전 독립 노드 → 12. 패키지 빌드 설정 → 13. 리팩터링 동등성 테스트
