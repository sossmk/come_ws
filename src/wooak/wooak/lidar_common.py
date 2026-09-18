#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 라이다(/lidar_array) 쓰는 노드들이 공통으로 쓰던 코드 모음
# lidar_drive, maze_drive, motor_control_node, choice* 에 복붙돼 있던 부분

import math

from wooak.dxl_common import STEER_CENTER, STEER_MIN, STEER_MAX  # noqa: F401 (여기서 같이 import 해서 씀)

# ===== YDLIDAR G6 기준 (lidar_array_node 가 1860개로 리샘플링) =====
NUM_BINS = 1860
N_MIN, N_MAX = 1800, 1900   # 이 범위 밖이면 다른 라이다/노드 데이터로 보고 무시

# ===== 조향 파라미터 =====
STEER_STEP = 3.125   # 인덱스 차이당 조향 변화량


def idx(*ranges):
    """(시작, 끝) 구간 여러 개를 인덱스 리스트 하나로. 예) idx((1610, 1860), (0, 250))"""
    out = []
    for a, b in ranges:
        out += list(range(a, b))
    return out


def valid_ranges(ranges, min_r=0.1):
    """너무 가깝거나(노이즈) 너무 멀거나 NaN 인 값은 20.0 으로 처리"""
    return [20.0 if (r <= min_r or r > 16.0 or math.isnan(r)) else r for r in ranges]


def zone_min(ranges_valid, indices):
    return min(ranges_valid[i] for i in indices)


def zone_argmin(ranges_valid, indices):
    return min(indices, key=lambda i: ranges_valid[i])


def clamp_steer(steer_pos):
    return max(min(steer_pos, STEER_MAX), STEER_MIN)
