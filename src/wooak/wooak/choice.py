#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 정면 좁은 구간(±10) 장애물 → /obstacle_direction
# 공통 로직은 choice_common.py

from wooak.choice_common import run
from wooak.lidar_common import idx


def main(args=None):
    run(topic='/obstacle_direction',
        center_indices=idx((1850, 1860), (0, 10)),
        min_r=0.2,        # 이 값 이하는 무시
        near_max=0.67,     # 이 거리 안이면 장애물(1)
        tag='')


if __name__ == '__main__':
    main()
