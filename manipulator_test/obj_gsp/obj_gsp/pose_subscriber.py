import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_srvs.srv import Trigger  # ROS 2 Service，用于请求最新的 Pose


class PoseSubscriber(Node):
    def __init__(self):
        super().__init__('pose_subscriber')
        self.latest_pose = None  # 存储最新的目标位置
        self.subscription = self.create_subscription(
            Pose,
            '/pose_topic',
            self.goal_pose_callback,
            10)

        # 提供一个 ROS 2 Service，让外部查询最新的目标位姿
        self.srv = self.create_service(Trigger, 'get_latest_pose', self.get_pose_service)

        self.get_logger().info('PoseSubscriber initialized, listening on /pose_topic')

    def goal_pose_callback(self, msg):
        """接收目标位姿并存储"""
        self.latest_pose = msg
        self.get_logger().info(f'Received new goal pose: x={msg.position.x}, y={msg.position.y}, z={msg.position.z}')

    def get_pose_service(self, request, response):
        """Service 回调函数，返回最新的目标位姿"""
        if self.latest_pose is None:
            response.success = False
            response.message = "No pose received yet."
        else:
            response.success = True
            response.message = f"{self.latest_pose.position.x} {self.latest_pose.position.y} {self.latest_pose.position.z}"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PoseSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
