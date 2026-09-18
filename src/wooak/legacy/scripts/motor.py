#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math
from dynamixel_sdk import *  # pip install dynamixel-sdk

class OdomFromDxl(Node):
    def __init__(self):
        super().__init__('odom_from_dxl')

        # ROS 퍼블리셔
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # 로봇 파라미터 (네 환경 맞춤)
        self.wheel_radius = 0.035    # 바퀴 반지름 [m]
        self.wheel_base = 0.16       # 좌우 바퀴 간격 [m]
        self.ticks_per_rev = 4096

        # Dynamixel 설정
        self.DEVICENAME = '/dev/ttyUSB1'
        self.BAUDRATE = 57600
        self.PROTOCOL_VERSION = 2.0
        self.DXL_LEFT_ID = 7   # 왼쪽 모터 ID
        self.DXL_RIGHT_ID = 9  # 오른쪽 모터 ID
        self.ADDR_PRESENT_POS = 132

        # 포트 초기화
        self.portHandler = PortHandler(self.DEVICENAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)
        if not self.portHandler.openPort():
            self.get_logger().error("Failed to open port")
        if not self.portHandler.setBaudRate(self.BAUDRATE):
            self.get_logger().error("Failed to set baudrate")

        # 엔코더 초기값 읽기
        self.prev_left = self.read_position(self.DXL_LEFT_ID)
        self.prev_right = self.read_position(self.DXL_RIGHT_ID)

        # 위치 상태
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()

        # 20Hz 주기 타이머
        self.create_timer(0.05, self.update_odom)

    def read_position(self, dxl_id):
        pos, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(
            self.portHandler, dxl_id, self.ADDR_PRESENT_POS)
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(self.packetHandler.getTxRxResult(dxl_comm_result))
        if dxl_error != 0:
            self.get_logger().error(self.packetHandler.getRxPacketError(dxl_error))
        return pos

    def update_odom(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt == 0.0:
            return

        # 현재 엔코더 값 읽기
        left_now = self.read_position(self.DXL_LEFT_ID)
        right_now = self.read_position(self.DXL_RIGHT_ID)

        # Δtick 계산 (4096 tick 래핑 처리)
        delta_left = (left_now - self.prev_left + self.ticks_per_rev) % self.ticks_per_rev
        delta_right = (right_now - self.prev_right + self.ticks_per_rev) % self.ticks_per_rev

        # tick → 거리 변환
        dist_left = 2 * math.pi * self.wheel_radius * (delta_left / self.ticks_per_rev)
        dist_right = 2 * math.pi * self.wheel_radius * (delta_right / self.ticks_per_rev)

        # 평균 이동거리와 회전각
        d = (dist_left + dist_right) / 2.0
        dth = (dist_right - dist_left) / self.wheel_base

        # 로봇 좌표 업데이트
        self.x += d * math.cos(self.th + dth/2.0)
        self.y += d * math.sin(self.th + dth/2.0)
        self.th += dth

        # 오도메트리 메시지 작성
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.th/2.0)
        odom.pose.pose.orientation.w = math.cos(self.th/2.0)

        odom.twist.twist.linear.x = d / dt
        odom.twist.twist.angular.z = dth / dt
        self.odom_pub.publish(odom)

        # TF 발행
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = math.sin(self.th/2.0)
        t.transform.rotation.w = math.cos(self.th/2.0)
        self.tf_broadcaster.sendTransform(t)

        # 이전값 업데이트
        self.prev_left = left_now
        self.prev_right = right_now
        self.last_time = now


def main(args=None):
    rclpy.init(args=args)
    node = OdomFromDxl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
