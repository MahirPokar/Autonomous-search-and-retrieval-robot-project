import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from std_srvs.srv import Trigger  # 用于请求最新的 Pose
import time


class ManipulatorControl(Node):
    def __init__(self):
        super().__init__('manipulator_control')
        self.bot = InterbotixManipulatorXS("px150", "arm", "gripper")

        # 创建一个客户端，请求 `pose_subscriber.py` 提供的 Service
        self.pose_client = self.create_client(Trigger, 'get_latest_pose')

        while not self.pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for pose subscriber service...')

        self.get_logger().info('ManipulatorControl initialized.')

    def get_latest_pose(self):
        """请求 `pose_subscriber.py` 获取最新目标位姿"""
        request = Trigger.Request()
        future = self.pose_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None and future.result().success:
            x, y, z = map(float, future.result().message.split())
            return x, y, z
        else:
            self.get_logger().warn("No valid pose received.")
            return None

    def execute_motion(self):
        """获取目标位姿，并执行机械臂运动"""
        pose = self.get_latest_pose()
        if pose is None:
            self.get_logger().warn("No goal pose received.")
            return

        x, y, z = pose
        self.get_logger().info(f'Moving to position: x={x}, y={y}, z={z}')

        # 执行机械臂运动
        self.bot.gripper.release()
        self.bot.arm.set_ee_pose_components(x=x, z=z, moving_time=6.0)
        time.sleep(1)  # 等待机械臂运动完成
        self.bot.gripper.grasp()
        self.bot.arm.go_to_sleep_pose()


def main(args=None):
    rclpy.init(args=args)
    node = ManipulatorControl()

    while rclpy.ok():
        node.execute_motion()  # 反复执行抓取任务
        time.sleep(5)  # 等待新目标

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
