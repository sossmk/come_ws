# legacy/

The build excludes this folder (`COLCON_IGNORE`). The files were moved here unchanged for reference and history.

| Item | Why it moved here |
|---|---|
| `dyna_lidar_bringup/` | Early RPLIDAR-era code. `launch/*` references `package='dyna_lidar_bringup'`, but no such package ever existed (it was only a module inside wooak). `lidar_array_node.py` defines `scan_callback` twice and uses 1440 bins, which is incompatible with the G6 (1860) nodes |
| `scripts/` | `motor.py` duplicates `odom_from_dxl.py`; `scan_to_cmdvel.py` can't run because its setup.py entry point has the wrong path |
| `launch/bringup_all, cmdvel_test, lidar_gap_steer, odom_test` | Reference packages/executables that don't exist (`dyna_lidar_bringup`, `lidar_gap_steer`) and hardcoded paths (`/home/kangsanmaru/...`). bringup_all's IndentationError was fixed |
| `config/lidar_gap_steer.launch.py` | Near-identical copy of `launch/lidar_gap_steer.launch.py` |
| `wooak_launch_dup/` | Copy of the launch files under `wooak/wooak/launch/` (setup.py never installed these) |
| `yolov8_train.py`, `yolov8_project/` | Desktop training script (Windows path `C:/...`). Not a ROS node |
| `wooak_backup.zip`, `package.xml.bak` | Old backups |
