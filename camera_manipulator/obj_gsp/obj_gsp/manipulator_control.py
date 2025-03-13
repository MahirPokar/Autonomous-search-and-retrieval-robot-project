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

        # Initialize the robotic arm
        self.bot = InterbotixManipulatorXS(
            robot_model='px150',
            group_name='arm',
            gripper_name='gripper',
        )

        # Subscribe to the target position
        self.subscription = self.create_subscription(
            Point,
            'target_position',
            self.target_callback,
            10
        )

        # Subscribe to the task completion status
        self.target_achieve_subscription = self.create_subscription(
            Bool,
            'target_achieve_status',
            self.target_achieve_callback,
            10
        )

        # Publish the grasping status
        self.status_publisher = self.create_publisher(Bool, 'grasping_status', 10)

        # Subscribe to the status of the car arriving at the box
        self.car_arrived_subscription = self.create_subscription(
            Bool,
            'car_arrived_at_box',
            self.car_arrived_callback,
            10
        )

        self.target_position = None
        self.object_pose = None  # Object pose
        self.processing = False  # Task execution status
        self.target_achieve = False  # Default: do not execute target task, controlled externally
        self.global_attempts = 0  # Overall task attempts (target detection + grasping)
        self.max_global_attempts = 3  # Max attempts for target detection and grasping

        self.get_logger().info("ManipulatorControl Node Started")

    def target_callback(self, target_msg):
        """Store target position after receiving target detection's xyz position and obtain object pose"""
        # Only store the target position
        self.target_position = target_msg
        self.object_pose = target_msg.w  # Get object pose using target_msg.w
        self.get_logger().info(f"Target position received: x={target_msg.x}, y={target_msg.y}, z={target_msg.z}")
        self.get_logger().info(f"Object pose received: {self.object_pose}")

    def start_grasping(self):
        """Perform grasping operation"""
        if self.target_position is None:
            self.get_logger().warn("No target to grasp, skipping.")
            return
        
        self.processing = True
        x, y, z = self.target_position.z + 0.11, self.target_position.x, -self.target_position.y + 0.04
        self.get_logger().info(f"Moving to target: x={x}, y={y}, z={z}")

        # Determine object pose and select the appropriate grasping method
        if self.object_pose == 0:
            self.get_logger().info("Grasping with pose 0")
            self.grasp_with_pose_0(x, y, z)
        elif self.object_pose == 1:
            self.get_logger().info("Grasping with pose 1")
            self.grasp_with_pose_1(x, y, z)
        elif self.object_pose == 2:
            self.get_logger().info("Grasping with pose 2")
            self.grasp_with_pose_2(x, y, z)
        else:
            self.get_logger().warn("Unknown object pose, skipping grasp.")

    def grasp_with_pose_0(self, x, y, z):
        """Grasp using pose 0"""
        self.bot.gripper.release()
        self.bot.arm.set_ee_pose_components(x=x, y=y, z=z, roll=0, pitch=0)
        time.sleep(1)
        self.bot.gripper.grasp()
        time.sleep(1)
        self.check_and_publish_grasping_status()

    def grasp_with_pose_1(self, x, y, z):
        """Grasp using pose 1"""
        self.bot.gripper.release()
        self.bot.arm.set_ee_pose_components(x=x, y=y, z=z, roll=0, pitch=1.47)
        time.sleep(1)
        self.bot.gripper.grasp()
        time.sleep(1)
        self.check_and_publish_grasping_status()

    def grasp_with_pose_2(self, x, y, z):
        """Grasp using pose 2"""
        self.bot.gripper.release()
        self.bot.arm.set_ee_pose_components(x=x, y=y, z=z, roll=1.47, pitch=1.47)
        time.sleep(1)
        self.bot.gripper.grasp()
        time.sleep(1)
        self.check_and_publish_grasping_status()

    def check_and_publish_grasping_status(self):
        """Check if grasping was successful and publish status"""
        if self.check_grasping_success():
            self.get_logger().info("Grasp successful!")
            self.publish_grasping_status(True)
            self.global_attempts = 0  # Reset attempts after successful grasp
            self.processing = False
        else:
            self.get_logger().info(f"Grasp failed, retrying in 2s ({self.global_attempts}/{self.max_global_attempts})")
            self.global_attempts += 1  # Increase attempt count
            if self.global_attempts >= self.max_global_attempts:
                self.get_logger().error("Max attempts reached, abandoning target.")
                self.publish_grasping_status(False)
                self.processing = False
                self.global_attempts = 0  # Reset attempts after reaching max attempts
            else:
                self.create_timer(2.0, lambda: self.start_grasping())  # Retry grasping after 2 seconds

    def car_arrived_callback(self, msg):
        """Perform placement operation when receiving the status of the car arriving at the box"""
        self.get_logger().info(f"Received car_arrived_at_box status: {msg.data}")

        if msg.data and not self.processing:  # Car has arrived at the box and no other tasks are ongoing
            self.get_logger().info("Car has arrived at the box. Proceeding with block placement.")
            self.processing = True
            self.place_in_box()  # Perform placement operation

    def place_in_box(self):
        """Place the block in the box"""
        if self.target_position is None:
            self.get_logger().warn("No target to place, skipping.")
            return
        
        x, y, z = self.target_position.z, self.target_position.x, self.target_position.y
        self.get_logger().info(f"Moving to box at: x={x}, y={y}, z={z}")

        # Move to box position
        self.bot.arm.set_ee_pose_components(x=x, y=y, z=z, roll=0, pitch=0)
        time.sleep(1)
        
        # Release the block
        self.bot.gripper.release()
        self.get_logger().info("Block placed in the box.")
        self.target_position = None  # Clear the target position
        self.processing = False  # Task completed, release task lock

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