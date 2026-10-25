# 실험 가이드 (come_ws)

오랜만에 이 저장소를 보는 팀원을 위한 문서다. **처음 켰을 때 칠 명령 → 카메라 확인 → 차선 인식 조정 → 주행** 순서로 정리했다.
마지막 점검일: 2026-10-25 (Jetson Orin Nano, ROS 2 Humble, USU Camera 4K)

---

## 0. 한눈에 보는 구조

```
USB 카메라 ──▶ usb_cam 노드 ──/image_raw──┬─▶ lane_detection_node        ──/lane_detection/Delta_x──────────┐
(/dev/video0)  (src/usb_cam)              ├─▶ left_lane_detection_node   ──/Left_lane_detection/Left_Delta_x─┤
                                          ├─▶ right_lane_detection_node  ──/Right_lane_detection/Right_Delta_x┤
                                          └─▶ stopline_find              ──/stop_line_detected────────────────┤
                                                                                                              ▼
라이다 ──▶ ydlidar ──/scan──▶ lidar_array / lidar_drive / maze_drive / choice* ─────────────────▶ main_drive_node (주행 FSM)
                                                                                                              │
                                                              /motor_command/angle, /motor_command/speed ◀───┘
                                                                                                              ▼
                                                                                                   m2ct (다이나믹셀 모터)
```

| 폴더 | 내용 |
|---|---|
| `src/wooak/wooak/` | **우리가 만든 노드 전부** (차선, 정지선, 라이다, 주행 FSM, 모터) |
| `src/wooak/launch/` | 실행 묶음 (`camall`, `competition`, `debug_view`, `lidarall`) |
| `src/usb_cam/` | 카메라 드라이버 (upstream 0.8.1 + **컨트롤 이름 버그 수정본**, 지우지 말 것) |
| `scripts/jetson_optimize.sh` | 주행 전 Jetson 성능 고정 |
| `config/usb_cam_params.yaml` | ⚠️ **어디서도 안 읽음.** 여기 고쳐도 반영 안 됨 |

---

## 1. 처음 켰을 때 (매번)

### 1-1. 터미널 환경
```bash
source /opt/ros/humble/setup.bash
source ~/come_ws/install/setup.bash
echo $ROS_LOCALHOST_ONLY        # 반드시 1 이어야 함
```

> ⚠️ **`ROS_LOCALHOST_ONLY`가 0이면 영상이 1~3초씩 밀린다.**
> 0이면 DDS가 영상(장당 약 900KB)을 Wi-Fi로 멀티캐스트해서 2Hz까지 떨어진다. 1이면 30Hz, 지연 45ms.
> Jetson의 `~/.bashrc`에는 `export ROS_LOCALHOST_ONLY=1`로 설정돼 있다.
> VNC/RustDesk 같은 원격 화면 접속은 이 값과 상관없으니 1로 둔 채 써도 된다.
> (0이 필요한 경우는 다른 PC에서 직접 `ros2 topic`/rviz를 쓸 때뿐이다.)
> 값을 바꿨으면 `ros2 daemon stop`을 한 번 실행할 것.

### 1-2. 빌드 (코드를 받았거나 고쳤을 때)
```bash
cd ~/come_ws                      # ⚠️ 반드시 루트에서. src/ 안에서 빌드하면 build/install이 중복 생성됨
colcon build                      # 전체
colcon build --packages-select wooak     # wooak만 (launch/노드 수정 후)
colcon build --packages-select usb_cam   # usb_cam만
source install/setup.bash
```

| 무엇을 고쳤나 | 빌드 필요? |
|---|---|
| `src/wooak/launch/*.launch.py` | **필요** (`--packages-select wooak`) |
| 노드 `.py` → `camall`로 실행 | 불필요 (`camall`은 소스 파일을 직접 실행) |
| 노드 `.py` → `competition`으로 실행 | **필요** |
| `src/usb_cam` | **필요** |

### 1-3. 하드웨어 확인
```bash
v4l2-ctl --list-devices          # USU Camera 4K → /dev/video0 (영상) + /dev/video1 (메타데이터, 사용 안 함)
ls /dev/ttyUSB*                  # 라이다(ttyUSB1), 모터 포트
pgrep -af "usb_cam|detection|stopline"   # 이전 실행이 남아 있으면 전부 끄고 시작
```

---

## 2. 카메라 테스트

### 2-1. 카메라만 띄우기
```bash
ros2 run usb_cam usb_cam_node_exe --ros-args \
  -p video_device:=/dev/video0 -p pixel_format:=mjpeg2rgb \
  -p image_width:=640 -p image_height:=480 -p framerate:=30.0 \
  -p autoexposure:=false -p exposure:=60 -p auto_white_balance:=false -p brightness:=-1
```
다른 터미널에서:
```bash
ros2 run rqt_image_view rqt_image_view      # /image_raw 선택
ros2 topic hz /image_raw                    # 정상: 약 30Hz (15fps 설정이면 15Hz)
v4l2-ctl -d /dev/video0 -C auto_exposure,exposure_time_absolute,brightness,backlight_compensation
#   정상: auto_exposure=1(수동), exposure=60, brightness=-5, backlight=0
```

### 2-2. 카메라 + 차선 노드 4개 + 뷰어
```bash
ros2 launch wooak camall.launch.py          # Jetson에는 alias `camall` 있음
```

### 2-3. 토픽 목록에 보이는 `/image_raw/compressed`, `compressedDepth`, `theora`
`image_transport`가 자동으로 만드는 토픽이고, **누가 구독하기 전엔 아무 일도 안 한다.** 무시해도 된다.
(rqt에서 고르면 그때부터 인코딩 부하가 생기니 `/image_raw`나 디버그 토픽을 볼 것)

---

## 3. 카메라 파라미터는 어디서 바꾸나

### 3-1. 실제로 적용되는 곳 (여기만 고치면 됨)
| 파일 | 언제 쓰나 |
|---|---|
| `src/wooak/launch/camall.launch.py` | 카메라 + 차선 인식 테스트 |
| `src/wooak/launch/competition.launch.py` | 대회 본 주행 (fps는 실행할 때 `camera_fps:=15.0`로 지정 가능) |

**두 파일의 값을 항상 같이 맞출 것.** 고친 뒤 `colcon build --packages-select wooak`.

| 파라미터 | 현재 값 | 의미 |
|---|---|---|
| `autoexposure` | `False` | 자동 노출 끔 (유광 시트 반사 대응) |
| `exposure` | `60` | 노출 시간. 범위 3~2047, 작을수록 어두움 |
| `auto_white_balance` | `False` | 자동 화이트밸런스 끔 (4000K 고정) |
| `brightness` | `-1` | usb_cam이 건드리지 않음 → 카메라 기본값 -5 유지 |
| `framerate` | camall 30 / competition 15 | |
| `backlight_compensation=0` | v4l2-ctl로 설정 | usb_cam에 이 파라미터가 없어서 launch에서 `v4l2-ctl`로 직접 설정 |

### 3-2. 값 찾을 때: 띄운 채로 바로 바꿔보기
```bash
v4l2-ctl -d /dev/video0 --list-ctrls                      # 조절 가능한 항목, 범위, 현재 값
v4l2-ctl -d /dev/video0 -c exposure_time_absolute=100     # 바로 반영됨
v4l2-ctl -d /dev/video0 -c brightness=0                   # -64 ~ 64
v4l2-ctl -d /dev/video0 -c focus_automatic_continuous=0   # 오토포커스 끄기
```
맞는 값을 찾으면 launch 파일에 옮겨 적는다. (v4l2-ctl로 바꾼 값은 재부팅하거나 카메라를 다시 꽂으면 초기화된다)

### 3-3. ⚠️ 왜 usb_cam을 우리 저장소에 들고 있나
apt로 설치된 usb_cam 0.8.1은 예전 커널 기준 컨트롤 이름(`exposure_auto`, `exposure_absolute` 등)을 써서,
**launch에 노출을 적어도 실제로는 무시된다** (로그에 `unknown control`이 찍힘).
`src/usb_cam/src/usb_cam_node.cpp`에서 이름을 수정했다. 커밋 `usb_cam: v4l2 컨트롤 이름을...`을 참고.
→ **`src/usb_cam`을 지우거나 apt 버전으로 바꾸면 노출 설정이 다시 무시된다.**

---

## 4. 차선 인식 코드는 어디서 바꾸나

전부 `src/wooak/wooak/`에 있다.

| 파일 | 역할 | 발행 토픽 |
|---|---|---|
| `lane_detection_node.py` | 중앙 차선 (흑백 기준) | `/lane_detection/Delta_x` |
| `left_lane_detection_node.py` | 왼쪽 노란 차선 | `/Left_lane_detection/Left_Delta_x` |
| `right_lane_detection_node.py` | 오른쪽 노란 차선 | `/Right_lane_detection/Right_Delta_x` |
| `stopline_find.py` | 흰색 정지선 | `/stop_line_detected` (Bool) |

`Delta_x` = (검출한 차선 중심 x) − (화면 중심 x). 단위는 픽셀이고, ±10 이내는 0으로 처리한다.

### 4-1. 처리 순서와 튜닝 위치 (`lane_detect()` 함수 안)
```
ROI 자르기 → (흑백 | 노란색 강조) → adaptiveThreshold → GaussianBlur → Canny → HoughLinesP → Delta_x
```

| 바꾸고 싶은 것 | 어디 | 현재 값 |
|---|---|---|
| **ROI(보는 영역) 세로 범위** | `__init__`의 `roi_start_y`, `roi_end_y` (+ `image_callback` 안에 같은 줄이 한 번 더 있음, 둘 다 고칠 것) | 화면 높이의 0.6 ~ 0.8 |
| ROI 가로 범위 | `lane_detect()` 맨 위 `roi_start_x`, `roi_end_x` | 중앙: 전체 / left: 50 ~ 절반 / right: 절반 ~ 끝-50 |
| **이진화 강도** | `cv2.adaptiveThreshold(..., 블록크기, C)` | 중앙/left/right 모두 `61, -20`, 정지선 `35, -20` |
| 노란색 강조 정도 | `cv2.multiply(b, 계수)` | left 1.6 / right 1.3 |
| 엣지 민감도 | `cv2.Canny(img, 100, 140)` | |
| 직선 검출 | `cv2.HoughLinesP(edge, 1, pi/180, 임계값30, 최소길이30, 최대간격15)` | |

**adaptiveThreshold의 C 값 읽는 법** (`THRESH_BINARY` 기준)
- C가 **음수**일수록 "주변보다 확실히 밝은 픽셀"만 흰색으로 남는다 → 반사·잡음이 줄지만, 흐린 차선은 놓칠 수 있다.
- C가 **양수**면 바닥까지 거의 전부 흰색이 된다 (+16일 때 화면의 99%가 흰색) → 차선 검출 불가. **양수는 쓰지 말 것.**
- 블록 크기는 홀수만 된다. 차선 폭보다 충분히 커야 한다.

### 4-2. 처리 결과 눈으로 보기 (rqt_image_view에서 선택)
| 토픽 | 내용 |
|---|---|
| `/lane_detection/processed_image` | 중앙: 원본 + ROI 박스 + 검출 선 + Delta_x |
| `/debug/blur` | 중앙: **adaptiveThreshold 결과**(이진화가 잘 됐는지 보기에 제일 좋음) |
| `/debug/edge` | 중앙: Canny 결과 (Hough에 실제로 들어가는 이미지) |
| `/right/debug/blur`, `/right/debug/edge` | 오른쪽 노드, 위와 같은 단계 |
| `/left_lane_detection/processed_image` | 왼쪽 (left는 디버그 토픽 코드가 주석 처리돼 있음) |
| `/stop/blur` | 정지선 이진화 결과 |

디버그 토픽은 **누가 볼 때만** 발행되니 대회 중 부하는 없다.

---

## 5. 주행 로직 (main_drive_node.py)

| 바꾸고 싶은 것 | 어디 |
|---|---|
| 조향 게인 | `self.steer_gain = 5.5` (조향 = `800 + steer_gain × Delta_x`, 800이 직진) |
| 기본 속도 | `self.speed_default = 200.0` |
| 미션별 동작 | `main_loop()`의 `if self.mode == ...` 분기 |
| 시작 미션 | 실행할 때 `start_mode:=N` |

미션 번호: `1` First_drive · `2` Second_drive · `3` OBSTACLE · `4` CONE · `5` PARKIN · `6` PARKING · `7` PARKOUT · `8` MAZE · `9` FINISH
(First_drive는 오른쪽 차선(`right_delta_x`) 기준으로 주행한다. 미션마다 어느 차선을 따라가는지 다르니 분기를 확인할 것)

---

## 6. 대회 주행

> ⚠️ `competition.launch.py`는 **모터까지 켠다. 실행하는 순간 차가 움직인다.** 바퀴를 띄우거나 트랙 위에서 실행할 것.

```bash
./scripts/jetson_optimize.sh                                     # 1회: 성능 모드 + 클럭 고정 (sudo)
ros2 launch wooak competition.launch.py start_mode:=1 camera_fps:=15.0
ros2 launch wooak debug_view.launch.py                           # (다른 터미널) 영상 확인이 필요할 때만
```
- rqt 같은 GUI는 CPU를 많이 써서 본 주행에는 띄우지 않는다.
- 원격 화면 접속(VNC/RustDesk)도 CPU를 쓰니 본 주행 전에는 끊는 게 좋다.

---

## 7. 문제 해결

| 증상 | 원인 / 해결 |
|---|---|
| 영상이 1~3초 늦게 나옴 | `echo $ROS_LOCALHOST_ONLY`가 1인지 확인 → 1로 바꾸고 `ros2 daemon stop` |
| usb_cam이 `terminate called after throwing an instance of 'char*'`로 바로 죽음 | 다른 프로세스가 카메라를 잡고 있음 → `pgrep -af usb_cam` 확인 후 종료 |
| 로그에 `unknown control 'exposure_auto'` | apt usb_cam이 실행 중 → `ros2 pkg prefix usb_cam`이 `~/come_ws/install/usb_cam`인지 확인, 아니면 `colcon build --packages-select usb_cam` 후 `source` |
| 화면이 하얗게 뜸 | `brightness`가 50으로 들어감 → launch에 `brightness: -1`, 그래도 그대로면 `v4l2-ctl -c brightness=-5` |
| 노드를 Ctrl+C로 껐는데 계속 떠 있음 | 파이썬 노드가 남는 경우가 있음 → `pgrep -af detection` 후 `kill` |
| `Unable to open camera calibration file` 에러 | 캘리브레이션 파일이 없다는 뜻. 영상에는 영향 없음, 무시 |
| 차선 처리 결과가 원본보다 느리게 보임 | 파이썬 노드 처리 속도 한계(10~18Hz). 정상 범위 |

---

## 8. Jetson에 있는 alias (`~/.bashrc`, kkk 계정)

| alias | 하는 일 |
|---|---|
| `camall` | `ros2 launch wooak camall.launch.py` |
| `lidarall` | 라이다 관련 launch |
| `start_cam` | usb_cam 단독 실행 (⚠️ 노출 40, `brightness` 설정 없음 → launch와 값이 다름) |
| `start_viewer` | rqt_image_view |
| `start_lane` / `left` / `right` / `start_stop` | 차선/정지선 노드 단독 실행 |
| `drive` / `motor` | 주행 FSM / 모터 노드 단독 실행 |
| `sb` | `source ~/.bashrc` |

---

## 9. 코드 올리기 (git)

`~/come_ws`가 `camera-fix` 브랜치에 연결돼 있다. (`main`은 초기 커밋만 있고, PR #1 `refactor/cleanup`은 별도 브랜치다)

```bash
cd ~/come_ws
git status
git add -A && git commit -m "무엇을 왜 바꿨는지"
git push        # Username: sossmk / Password: GitHub 토큰 (비밀번호 아님, 붙여넣기는 Ctrl+Shift+V)
```
`build/`, `install/`, `log/`, `src/yolo_ros/`는 `.gitignore`로 제외돼 있다.

---

## 10. 남은 할 일 (2026-10-25 기준)

- [ ] **트랙(유광 시트)에서 노출 값 튜닝.** 노출/AWB 고정이 이번에 처음 제대로 적용됐다. 예전 임계값들도 이 기준으로 다시 볼 것
- [ ] **left/right의 adaptiveThreshold C 값 검증.** 중앙과 같은 `-20`을 쓰지만, left/right는 노란색 강조 이미지라 맞지 않을 수 있다. 차선이 끊기면 `-10` 정도로 올려볼 것
- [ ] left 노드 디버그 토픽 발행 코드 살리기 (현재 주석 처리)
- [ ] 반사가 여전하면 Canny 대신 **Sobel X** 검토 (세로 차선은 살리고 가로 반사 경계는 줄임. 정지선 노드엔 쓰지 말 것)
- [ ] 차선 노드 처리 속도 개선 (현재 10~18Hz, 파이썬 CPU 병목)
- [ ] 정리: `config/usb_cam_params.yaml`(안 쓰임), `src/wooak/wooak/launch/`(설치 안 되는 옛 사본)
