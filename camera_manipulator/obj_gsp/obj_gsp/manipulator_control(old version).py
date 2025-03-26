#!/usr/bin/env python3

import rclpy  
from rclpy.node import Node  
from geometry_msgs.msg import Point 
from std_msgs.msg import String, Bool  
from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup  
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS  
import math  
import time  

class ManipulatorControl(Node):
    def __init__(self):
        super().__init__('manipulator_control')  # Create a ROS 2 node named 'manipulator_control'

        # Initialize the manipulator
        self.bot = InterbotixManipulatorXS(
            robot_model='px150',  # Manipulator model
            group_name='arm',  # Control group for the manipulator
            gripper_name='gripper',  # Gripper name
        )

        # Subscribe to target position
        self.subscription = self.create_subscription(
            Point, 'target_position', self.target_callback, 10
        )

        # Subscribe to target label 
        self.target_label_subscription = self.create_subscription(
            String, 'target_label', self.label_callback, 10
        )

        # Subscribe to target achievement status 
        self.target_achieve_subscription = self.create_subscription(
            Bool, 'target_achieve_status', self.target_achieve_callback, 10
        )

        # Publisher for grasping status
        self.status_publisher = self.create_publisher(Bool, 'grasping_status', 10)

        # Subscribe to car arrival at box position
        self.car_arrived_subscription = self.create_subscription(
            Bool, 'car_arrived_at_box', self.car_arrived_callback, 10
        )

        # Initialize state variables
        self.target_position = None  # Target position
        self.object_pose = None  # Object orientation 
        self.processing = False  # Flag indicating if a task is in progress
        self.target_achieve = False  # Whether to execute the target task
        self.global_attempts = 0  # Total attempts (target detection + grasping)
        self.max_global_attempts = 3  # Maximum number of attempts

        self.get_logger().info("ManipulatorControl node started")

    def target_callback(self, target_msg):
        self.target_position = target_msg  # Store target position
        self.get_logger().info(f"Target position received: x={target_msg.x}, y={target_msg.y}, z={target_msg.z}")

    def label_callback(self, label_msg):
        """Receive object label and convert it to corresponding pose number"""
        self.get_logger().info(f"Received label information: {label_msg.data}")
        try:
            self.object_pose = int(label_msg.data.split('_')[-1])
        except ValueError:
            self.get_logger().warn(f"Unknown label information: {label_msg.data}")
            self.object_pose = None

    def target_achieve_callback(self, msg):
        """Upon receiving target achievement status, execute grasping behavior"""
        if msg.data:
            self.get_logger().info("Target position reached, starting grasping action.")
            self.start_grasping()

    def start_grasping(self):
        if self.target_position is None:
            self.get_logger().warn("No target position, skipping grasping.")
            return

        self.processing = True
        x, y, z = self.target_position.x, self.target_position.y, self.target_position.z
        self.get_logger().info(f"Moving to target position: x={x}, y={y}, z={z}")

        if self.object_pose == 0:
            self.grasp_with_pose_0(x, y, z)
        elif self.object_pose == 1:
            self.grasp_with_pose_1(x, y, z)
        elif self.object_pose == 2:
            self.grasp_with_pose_2(x, y, z)
        else:
            self.get_logger().warn("Unknown object pose, skipping grasping.")

    def grasp_with_pose_0(self, x, y, z):
        self.bot.gripper.release()
        self.bot.arm.set_ee_pose_components(x=x, y=y, z=z, roll=0, pitch=0)
        time.sleep(1)
        self.bot.gripper.grasp()
        time.sleep(1)
        self.check_and_publish_grasping_status()

    def grasp_with_pose_1(self, x, y, z):
        self.bot.gripper.release()
        self.bot.arm.set_ee_pose_components(x=x, y=y, z=z, roll=0, pitch=1.47)
        time.sleep(1)
        self.bot.gripper.grasp()
        time.sleep(1)
        self.check_and_publish_grasping_status()

    def grasp_with_pose_2(self, x, y, z):
        self.bot.gripper.release()
        self.bot.arm.set_ee_pose_components(x=x, y=y, z=z, roll=1.47, pitch=1.47)
        time.sleep(1)
        self.bot.gripper.grasp()
        time.sleep(1)
        self.check_and_publish_grasping_status()

    def check_and_publish_grasping_status(self):
        if self.check_grasping_success():
            self.get_logger().info("Grasp successful!")
            self.publish_grasping_status(True)
            self.global_attempts = 0
            self.processing = False
        else:
            self.global_attempts += 1
            if self.global_attempts >= self.max_global_attempts:
                self.get_logger().error("Maximum number of attempts reached, abandoning target.")
                self.publish_grasping_status(False)
                self.processing = False
                self.global_attempts = 0
            else:
                self.create_timer(2.0, self.start_grasping)

    def car_arrived_callback(self, msg):
        if msg.data and not self.processing:
            self.get_logger().info("Car has arrived at the box, starting block placement.")
            self.processing = True
            self.place_in_box()

    def place_in_box(self):
        if self.target_position is None:
            self.get_logger().warn("No target to place, skipping.")
            return

        x, y, z = self.target_position.x, self.target_position.y, self.target_position.z
        self.bot.arm.set_ee_pose_components(x=x, y=y, z=z, roll=0, pitch=0)
        time.sleep(1)
        self.bot.gripper.release()
        self.get_logger().info("Block placed in the box.")
        self.target_position = None
        self.processing = False


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
