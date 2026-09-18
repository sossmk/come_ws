import rclpy
from rclpy.node import Node

class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')
        self.declare_parameter('left_id', 7)
        self.declare_parameter('right_id', 9)
        self.declare_parameter('steer_id', 8)
        self.declare_parameter('port', '/dev/ttyUSB1')
        self.declare_parameter('baudrate', 57600)
        self.get_logger().info('Motor node started (stub).')

def main():
    rclpy.init()
    node = MotorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
