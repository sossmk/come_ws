#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from dynamixel_sdk import *  # pip install dynamixel-sdk


class OdomFromDxl(Node):
    def __init__(self):
        super().__init__('odom_from_dxl')

        # ====== Parameters (ros2 launch/CLI에서 override 가능) ======
        self.declare_parameter('device', '/dev/ttyUSB1')
        self.declare_parameter('baudrate', 57600)
        self.declare_parameter('protocol_version', 2.0)
        self.declare_parameter('left_id', 7)
        self.declare_parameter('right_id', 9)
        self.declare_parameter('ticks_per_rev', 4096)
        self.declare_parameter('wheel_radius', 0.035)  # [m]
        self.declare_parameter('wheel_base', 0.16)     # [m]
        self.declare_parameter('frame_odom', 'odom')
        self.declare_parameter('frame_base', 'base_link')
        self.declare_parameter('pub_rate', 20.0)       # Hz
        self.declare_parameter('reverse_left', False)  # ← 롤백: 기본 False
        self.declare_parameter('reverse_right', False)

        self.device         = self.get_parameter('device').get_parameter_value().string_value
        self.baudrate       = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.protocol_ver   = float(self.get_parameter('protocol_version').get_parameter_value().double_value)
        self.left_id        = self.get_parameter('left_id').get_parameter_value().integer_value
        self.right_id       = self.get_parameter('right_id').get_parameter_value().integer_value
        self.ticks_per_rev  = self.get_parameter('ticks_per_rev').get_parameter_value().integer_value
        self.wheel_radius   = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_base     = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.frame_odom     = self.get_parameter('frame_odom').get_parameter_value().string_value
        self.frame_base     = self.get_parameter('frame_base').get_parameter_value().string_value
        self.pub_rate       = self.get_parameter('pub_rate').get_parameter_value().double_value
        self.reverse_left   = self.get_parameter('reverse_left').get_parameter_value().bool_value
        self.reverse_right  = self.get_parameter('reverse_right').get_parameter_value().bool_value

        # ====== ROS pubs ======
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_br = TransformBroadcaster(self)

        # ====== Dynamixel setup (read-only) ======
        self.ADDR_PRESENT_POS = 132
        self.port = PortHandler(self.device)
        self.packet = PacketHandler(self.protocol_ver)

        if not self.port.openPort():
            self.get_logger().fatal(f'Failed to open port: {self.device}')
            raise SystemExit(1)
        if not self.port.setBaudRate(self.baudrate):
            self.get_logger().fatal(f'Failed to set baudrate: {self.baudrate}')
            raise SystemExit(1)

        # 초기 엔코더 값
        self.prev_left = self._read_pos(self.left_id)
        self.prev_right = self._read_pos(self.right_id)

        # 상태
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()

        # 주기 타이머
        period = 1.0 / max(self.pub_rate, 1.0)
        self.create_timer(period, self._update)

        self.get_logger().info(
            f'Odometry from DXL started: dev={self.device}, baud={self.baudrate}, '
            f'IDs(L,R)=({self.left_id},{self.right_id}), ticks/rev={self.ticks_per_rev}'
        )

    # --- helpers ---
    def _read_pos(self, dxl_id: int) -> int:
        pos, comm_result, dxl_error = self.packet.read4ByteTxRx(self.port, dxl_id, self.ADDR_PRESENT_POS)
        if comm_result != COMM_SUCCESS:
            self.get_logger().error(f'ID {dxl_id} comm: {self.packet.getTxRxResult(comm_result)}')
        if dxl_error != 0:
            self.get_logger().error(f'ID {dxl_id} error: {self.packet.getRxPacketError(dxl_error)}')
        return int(pos) if pos is not None else 0

    def _wrap_delta(self, now: int, prev: int) -> int:
        # 모듈로 wrap 고려하여 Δtick 계산 (0..ticks_per_rev-1)
        raw = (now - prev) % self.ticks_per_rev
        if raw > self.ticks_per_rev / 2:
            raw -= self.ticks_per_rev
        return raw

    # --- main loop ---
    def _update(self):
        now_time = self.get_clock().now()
        dt = (now_time - self.last_time).nanoseconds / 1e9
        if dt <= 0.0:
            return

        l_now = self._read_pos(self.left_id)
        r_now = self._read_pos(self.right_id)

        dl = self._wrap_delta(l_now, self.prev_left)
        dr = self._wrap_delta(r_now, self.prev_right)
        if self.reverse_left:
            dl = -dl
        if self.reverse_right:
            dr = -dr

        # tick -> distance
        tick_to_meter = 2.0 * math.pi * self.wheel_radius / float(self.ticks_per_rev)
        dist_l = dl * tick_to_meter
        dist_r = dr * tick_to_meter

        d   = (dist_l + dist_r) / 2.0
        dth = (dist_r - dist_l) / self.wheel_base

        # pose update (1st-order integration, midpoint heading)
        self.x  += d * math.cos(self.th + dth * 0.5)
        self.y  += d * math.sin(self.th + dth * 0.5)
        self.th += dth

        # publish odom
        odom = Odometry()
        odom.header.stamp = now_time.to_msg()
        odom.header.frame_id = self.frame_odom
        odom.child_frame_id  = self.frame_base

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.th / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.th / 2.0)

        odom.twist.twist.linear.x  = d / dt
        odom.twist.twist.angular.z = dth / dt
        self.odom_pub.publish(odom)

        # publish TF
        t = TransformStamped()
        t.header.stamp = now_time.to_msg()
        t.header.frame_id = self.frame_odom
        t.child_frame_id  = self.frame_base
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = odom.pose.pose.orientation.z
        t.transform.rotation.w = odom.pose.pose.orientation.w
        self.tf_br.sendTransform(t)

        # update prev
        self.prev_left  = l_now
        self.prev_right = r_now
        self.last_time  = now_time


def main(args=None):
    rclpy.init(args=args)
    node = OdomFromDxl()
    try:
        rclpy.spin(node)
    finally:
        try:
            node.port.closePort()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

