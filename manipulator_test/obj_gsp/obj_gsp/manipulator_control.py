#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import math
import time


class ManipulatorControl(Node):
    def __init__(self):
        super().__init__('manipulator_control')

        # 初始化机械臂
        self.bot = InterbotixManipulatorXS(
            robot_model='px150',
            group_name='arm',
            gripper_name='gripper',
        )
        robot_startup()

        # 订阅目标位置
        self.subscription = self.create_subscription(
            Point,
            'target_position',
            self.target_callback,
            10
        )

        self.processing = False  # 任务执行状态
        self.last_target_position = None  # 记录上一个目标的位置
        self.position_threshold = 0.02  # 2cm 作为目标变化的阈值
        self.achieve_traget = True # 目标到达标志位

        self.get_logger().info("ManipulatorControl Node Started")

    def target_callback(self, msg):
        # 只有 achieve_traget 为 True 时才执行
        if not self.achieve_traget:
            self.get_logger().info("Achieve target flag is False, skipping.")
            return
        
        """收到目标检测的 xyz 位置后控制机械臂移动"""
        if self.processing:
            self.get_logger().info("Manipulator is busy, skipping new target.")
            return
        
        x, y, z = msg.z+0.11, msg.x, -msg.y+0.04
        new_target = (x, y, z)

        # 允许重复抓取，只有当目标位置变化足够大时才会更新目标点
        if self.last_target_position:
            dist = math.dist(new_target, self.last_target_position)
            if dist < self.position_threshold:
                self.get_logger().info(f"Target position change too small ({dist:.4f}m), skipping.")
                return

        self.last_target_position = new_target
        self.processing = True  # 标记机械臂正在执行任务
        self.get_logger().info(f"Processing new target: x={x}, y={y}, z={z}")

        try:
            # 控制机械臂抓取目标
            self.bot.gripper.release()
            self.bot.arm.set_ee_pose_components(x=x, y=y, z=z, roll=0, pitch=1.47)
            time.sleep(1)  # 等待机械臂运动完成
            self.bot.gripper.grasp()
            time.sleep(1)  # 等待机械臂运动完成
            self.bot.arm.go_to_home_pose()
            time.sleep(1)  # 等待机械臂运动完成
            self.bot.arm.go_to_sleep_pose()
            self.bot.gripper.release()
            self.get_logger().info("Target reached and object grasped.")
            robot_shutdown()
        except Exception as e:
            self.get_logger().error(f"Manipulator execution failed: {e}")
            robot_shutdown()
        self.processing = False  # 任务完成，允许接收新目标



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
