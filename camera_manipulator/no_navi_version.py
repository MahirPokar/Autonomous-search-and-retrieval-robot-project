#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import time
from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS


class ManipulatorControl(Node):
    def __init__(self):
        super().__init__('manipulator_control')

        self.bot = InterbotixManipulatorXS(
            robot_model='px150',
            group_name='arm',
            gripper_name='gripper',
        )

        self.subscription = self.create_subscription(
            String,
            'target_position',
            self.target_info_callback,
            10
        )

        self.status_publisher = self.create_publisher(Bool, 'grasping_status', 10)

        self.processing = False
        self.last_target = None  # (x, y, z, label)
        self.tolerance = 0.01  # 容差：米

        self.get_logger().info("ManipulatorControl Node Started")

    def get_grasping_orientation(self, tag):
        if "_0" in tag:
            return 0.0, 0.0
        elif "_1" in tag:
            return 0, 1.47
        elif "_2" in tag:
            return 1.47, 1.47
        else:
            return None

    def check_grasping_success(self):
        """使用 effort + finger_position 判断是否抓住了物体"""
        gripper_effort = self.bot.gripper.get_gripper_effort()
        finger_position = self.bot.gripper.get_finger_position()

        effort_threshold = 30.0       # 力阈值（%）
        position_threshold = 0.02     # 开口阈值（米）

        self.get_logger().info(f"Gripper effort: {gripper_effort:.4f}, Finger position: {finger_position:.4f}")

        return (gripper_effort > effort_threshold) and (finger_position < position_threshold)

    def publish_grasping_status(self, success):
        msg = Bool()
        msg.data = success
        self.status_publisher.publish(msg)
        self.get_logger().info(f"Published grasping status: {success}")

    def target_info_callback(self, msg):
        if self.processing:
            self.get_logger().info("Manipulator is busy, skipping new target.")
            return

        try:
            data = dict(item.split(":") for item in msg.data.strip().split(" "))
            x = float(data['x'])
            y = float(data['y'])
            z = float(data['z'])
            label = data['label']
        except Exception as e:
            self.get_logger().error(f"Failed to parse message: {msg.data}")
            return

        # 容差比较是否是新目标
        if self.last_target is not None:
            lx, ly, lz, llabel = self.last_target
            if (abs(x - lx) < self.tolerance and
                abs(y - ly) < self.tolerance and
                abs(z - lz) < self.tolerance and
                label == llabel):
                self.get_logger().info("Target too close to last one, skipping.")
                return

        orientation = self.get_grasping_orientation(label)
        if orientation is None:
            self.get_logger().warning(f"Unrecognized label: {label}, skipping.")
            return

        roll, pitch = orientation
        tx, ty, tz = z + 0.13, x, -y + 0.02

        self.processing = True
        self.get_logger().info(f"Processing NEW target {label}: x={tx}, y={ty}, z={tz}, roll={roll}, pitch={pitch}")

        try:
            self.bot.gripper.release()
            self.bot.arm.set_ee_pose_components(x=tx, y=ty, z=tz, roll=roll, pitch=pitch)
            time.sleep(1)

            self.bot.gripper.grasp()
            time.sleep(1)

            # ✅ 抓取判断
            success = self.check_grasping_success()
            if success:
                self.get_logger().info("Grasp successful!")
            else:
                self.get_logger().info("Grasp failed.")

            self.publish_grasping_status(success)

            self.bot.arm.go_to_home_pose()
            time.sleep(1)
            self.bot.arm.go_to_sleep_pose()
            self.bot.gripper.release()

            self.last_target = (x, y, z, label)
        except Exception as e:
            self.get_logger().error(f"Error during manipulation: {e}")
        finally:
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
