#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 다이나믹셀(X-series, Protocol 2.0) 공통 설정
# m2ct.py, motor_control_node.py 에 각각 복붙돼 있던 값들

# ===== 모터 ID =====
ID_STEER = 7
ID_LEFT = 8
ID_RIGHT = 9

BAUDRATE = 57600
PROTOCOL_VERSION = 2.0

# ===== 제어 테이블 주소 =====
ADDR_TORQUE_ENABLE = 64
ADDR_OPERATING_MODE = 11
ADDR_GOAL_VELOCITY = 104
ADDR_GOAL_POSITION = 116

# ===== 동작 모드 =====
MODE_VELOCITY = 1
MODE_POSITION = 3
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

VEL_LIMIT = 200

# ===== 조향 (position 틱) =====
STEER_CENTER = 800
STEER_MIN = STEER_CENTER - 500   # 300
STEER_MAX = STEER_CENTER + 500   # 1300
