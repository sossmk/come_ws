#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray   # 원래 `import Float32` → ImportError 로 노드가 안 떴음
from dynamixel_sdk import PortHandler, PacketHandler

from wooak.dxl_common import (BAUDRATE, ADDR_TORQUE_ENABLE, ADDR_OPERATING_MODE,
                              ADDR_GOAL_VELOCITY, ADDR_GOAL_POSITION, TORQUE_ENABLE,
                              MODE_VELOCITY, MODE_POSITION, ID_STEER, ID_LEFT, ID_RIGHT, VEL_LIMIT)
from wooak.lidar_common import (STEER_CENTER, STEER_STEP, N_MIN, N_MAX, idx,
                                valid_ranges, zone_min, zone_argmin, clamp_steer)

# ======== Dynamixel 설정 ========
DEVICENAME = '/dev/ttyUSB1'
# =================================


class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')

        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(2.0)

        if not self.portHandler.openPort():
            self.get_logger().error("❌ 포트를 열 수 없습니다.")
            return
        if not self.portHandler.setBaudRate(BAUDRATE):
            self.get_logger().error("❌ 보드레이트 설정 실패")
            return

        # 각 모터 초기 설정
        for motor_id in [ID_LEFT, ID_RIGHT]:
            self.packetHandler.write1ByteTxRx(self.portHandler, motor_id, ADDR_OPERATING_MODE, MODE_VELOCITY)
            self.packetHandler.write1ByteTxRx(self.portHandler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

        self.packetHandler.write1ByteTxRx(self.portHandler, ID_STEER, ADDR_OPERATING_MODE, MODE_POSITION)
        self.packetHandler.write1ByteTxRx(self.portHandler, ID_STEER, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

        self.sub = self.create_subscription(Float32MultiArray, '/lidar_array', self.lidar_callback, 10)
        self.get_logger().info("✅ MotorControlNode started (YDLIDAR G6 mode active)")
        self.lidar_angle_pub= self.create_publisher(Float32,'lidar_angle/delta_x',10)

    def lidar_callback(self, msg):
        if not msg.data:
            return

        ranges = msg.data
        n = len(ranges)

        # ===== YDLIDAR G6 데이터 유효성 =====
        if n < N_MIN or n > N_MAX:
            self.get_logger().warn(f"⚠️ 예상 인덱스(약 1860)와 다름: {n}")
            return

        # ===== 구역 설정 (0~1860 인덱스 기준) =====
        right_indices  = idx((1550, 1700))
        center_indices = idx((1700, 1860), (0, 100))
        left_indices   = idx((100, 250))

        # ===== 거리 유효성 검증 =====
        ranges_valid = valid_ranges(ranges)

        # ===== 각 영역 최소값 =====
        right_min_val  = zone_min(ranges_valid, right_indices)
        center_min_val = zone_min(ranges_valid, center_indices)
        left_min_val   = zone_min(ranges_valid, left_indices)

        # ===== 최소값 인덱스 =====
        right_min_idx  = zone_argmin(ranges_valid, right_indices)
        left_min_idx   = zone_argmin(ranges_valid, left_indices)

        # ===== 기본 설정 =====
        steer_pos = STEER_CENTER
        v_left = v_right = 200
        state = "🟢 Straight"

        # ===== 전방 하위 영역 =====
        front_left_min  = zone_min(ranges_valid, idx((1750, 1860)))
        front_right_min = zone_min(ranges_valid, idx((0, 110)))

        # ===== 주행 제어 로직 =====
        if (right_min_val < 0.5 or center_min_val < 0.6 or left_min_val < 0.5):
            # 전방(center) 장애물이 가까움
            if center_min_val < 0.6:
                if front_left_min > front_right_min:
                    steer_pos = 1300
                    state = f"⬅️ Hard Left | frontL={front_left_min:.2f} > frontR={front_right_min:.2f}"
                elif front_left_min < front_right_min:
                    steer_pos = 300
                    state = f"➡️ Hard Right | frontL={front_left_min:.2f} < frontR={front_right_min:.2f}"

            # 왼쪽이 가까운 경우
            elif left_min_val < right_min_val and left_min_val < 0.5:
                offset = (251 - left_min_idx) * STEER_STEP
                steer_pos = STEER_CENTER + offset
                state = f"↩️ Turn Right | idx={left_min_idx} (near 250)"

            # 오른쪽이 가까운 경우
            elif right_min_val < left_min_val and right_min_val < 0.5:
                offset = (right_min_idx - 1549) * STEER_STEP
                steer_pos = STEER_CENTER - offset
                state = f"↪️ Turn Left | idx={right_min_idx} (near 1550)"

            else:
                steer_pos = STEER_CENTER
                state = "🟢 Straight Drive"

        else:
            steer_pos = STEER_CENTER
            state = "🟢 Straight Drive"

        # ===== 조향 한계 제한 =====
        steer_pos = clamp_steer(steer_pos)

        # ===== 오른쪽 모터 반전 =====
        v_right = -v_right

        # ===== 명령 전송 =====
        self.packetHandler.write4ByteTxRx(self.portHandler, ID_STEER, ADDR_GOAL_POSITION, int(steer_pos))
        for motor_id, vel in zip([ID_LEFT, ID_RIGHT], [v_left, v_right]):
            vel = max(min(vel, VEL_LIMIT), -VEL_LIMIT)
            self.packetHandler.write4ByteTxRx(
                self.portHandler, motor_id, ADDR_GOAL_VELOCITY,
                int(vel) if vel >= 0 else int(vel + 2**32)
            )
        
        self.lidar_angle_pub.publish(Float32(data=float(steer_pos)))

        # ===== 터미널 출력 =====
        self.get_logger().info(
            f"{state} | steer={steer_pos} | "
            f"L={left_min_val:.2f} R={right_min_val:.2f} C={center_min_val:.2f} | "
            f"frontL={front_left_min:.2f} frontR={front_right_min:.2f}"
        )

    def destroy_node(self):
        for motor_id in [ID_LEFT, ID_RIGHT]:
            self.packetHandler.write4ByteTxRx(self.portHandler, motor_id, ADDR_GOAL_VELOCITY, 0)
            self.packetHandler.write1ByteTxRx(self.portHandler, motor_id, ADDR_TORQUE_ENABLE, 0)
        self.packetHandler.write1ByteTxRx(self.portHandler, ID_STEER, ADDR_TORQUE_ENABLE, 0)
        self.portHandler.closePort()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
