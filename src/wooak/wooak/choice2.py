#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 정면 아주 좁은 구간(±5) 장애물 → /obstacle_direction2
# 공통 로직은 choice_common.py

from wooak.choice_common import run
from wooak.lidar_common import idx


def main(args=None):
    run(topic='/obstacle_direction2',
        center_indices=idx((1855, 1860), (0, 5)),
        min_r=0.1,        # 이 값 이하는 무시
        near_max=0.9,     # 이 거리 안이면 장애물(1)
        tag='2')


if __name__ == '__main__':
    main()
