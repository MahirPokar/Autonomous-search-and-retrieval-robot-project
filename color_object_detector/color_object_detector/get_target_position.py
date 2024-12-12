import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point


class TargetPositionSubscriber(Node):
    def __init__(self):
        super().__init__('get_target_position')

        # 创建订阅器
        self.subscription = self.create_subscription(
            Point,
            'target_position',
            self.target_position_callback,
            10
        )
        self.subscription  # 防止未使用警告
        self.get_logger().info("Target Position Subscriber Node Started")

    def target_position_callback(self, msg):
        # 打印接收到的目标位置
        self.get_logger().info(f"Received Target Position: x={msg.x}, y={msg.y}, z={msg.z}")


def main(args=None):
    rclpy.init(args=args)
    node = TargetPositionSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
