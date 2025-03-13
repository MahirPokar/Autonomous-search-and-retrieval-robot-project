#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
import time
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS


class ManipulatorControl(Node):
    def __init__(self):
        super().__init__('manipulator_control')

        # 初始化机械臂
        self.bot = InterbotixManipulatorXS(
            robot_model='px150',
            group_name='arm',
            gripper_name='gripper',
        )

        # 订阅目标位置
        self.subscription = self.create_subscription(
            Point,
            'target_position',
            self.target_callback,
            10
        )

        # 发布抓取状态
        self.status_publisher = self.create_publisher(Bool, 'grasping_status', 10)

        self.processing = False  # 任务执行状态
        self.global_attempts = 0  # 整体任务尝试次数（目标检测 + 抓取）
        self.max_global_attempts = 3  # 目标检测和抓取最多尝试次数

        self.get_logger().info("ManipulatorControl Node Started")

    def publish_grasping_status(self, success):
        """发布抓取状态"""
        status_msg = Bool()
        status_msg.data = success
        self.status_publisher.publish(status_msg)
        self.get_logger().info(f"Published grasping status: {success}")

    def check_grasping_success(self):
        """使用 力 (`effort`) + 开口 (`finger_position`) 组合判断抓取成功"""
        gripper_effort = self.bot.gripper.get_gripper_effort()  
        finger_position = self.bot.gripper.get_finger_position()

        effort_threshold = 30.0  
        position_threshold = 0.02  

        self.get_logger().info(f"Gripper effort: {gripper_effort:.4f}, Finger position: {finger_position:.4f}")

        return (gripper_effort > effort_threshold) and (finger_position < position_threshold)

    def target_callback(self, target_msg):
        """收到目标检测的 xyz 位置后控制机械臂移动"""
        if self.processing:
            self.get_logger().info("Manipulator is busy, skipping new target.")
            return

        if self.global_attempts >= self.max_global_attempts:
            self.get_logger().error("Max attempts reached, abandoning target.")
            self.publish_grasping_status(False)
            self.global_attempts = 0
            self.processing = False
            return

        self.global_attempts += 1
        self.processing = True

        x, y, z = target_msg.z + 0.11, target_msg.x, -target_msg.y + 0.04
        self.get_logger().info(f"Attempt {self.global_attempts}/{self.max_global_attempts} to process target: x={x}, y={y}, z={z}")

        self.bot.gripper.release()
        self.bot.arm.set_ee_pose_components(x=x, y=y, z=z, roll=0, pitch=1.47)
        time.sleep(1)

        self.bot.gripper.grasp()
        time.sleep(1)

        if self.check_grasping_success():
            self.get_logger().info("Grasp successful!")
            self.publish_grasping_status(True)
            self.global_attempts = 0
        else:
            self.get_logger().info(f"Grasp failed, retrying in 2s ({self.global_attempts}/{self.max_global_attempts})")
            self.processing = False
            self.create_timer(2.0, lambda: self.target_callback(target_msg))
            return

        self.bot.arm.go_to_home_pose()
        time.sleep(1)
        self.bot.arm.go_to_sleep_pose()
        self.bot.gripper.release()

        self.processing = False
        self.get_logger().info("Ready for next target.")


def main(args=None):
    rclpy.init(args=args)
    node = ManipulatorControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down node...")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

