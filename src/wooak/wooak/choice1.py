#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 정면 넓은 구간(±250) 장애물 → /obstacle_direction1
# 공통 로직은 choice_common.py

from wooak.choice_common import run
from wooak.lidar_common import idx


def main(args=None):
    run(topic='/obstacle_direction1',
        center_indices=idx((1610, 1860), (0, 250)),
        min_r=0.1,        # 이 값 이하는 무시
        near_max=1.2,     # 이 거리 안이면 장애물(1)
        tag='1')


if __name__ == '__main__':
    main()
