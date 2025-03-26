#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS


class ManipulatorControl(Node):
    def __init__(self):
        super().__init__('manipulator_control')

        # 初始化 Interbotix 机械臂
        self.bot = InterbotixManipulatorXS(
            robot_model='px150',
            group_name='arm',
            gripper_name='gripper',
        )

        # 订阅统一的目标信息（xyz + label）
        self.subscription = self.create_subscription(
            String,
            'target_position',
            self.target_info_callback,
            10
        )

        self.processing = False
        self.last_target = None  # (x, y, z, label)
        self.tolerance = 0.01    # 坐标误差容差，单位：米
        self.get_logger().info("ManipulatorControl Node Started")

    def get_grasping_orientation(self, tag):
        """根据标签提取编号并选择对应的 Roll 和 Pitch"""
        if "_0" in tag:
            return 0.0, 0.0  # 正抓
        elif "_1" in tag:
            return 0, 1.47  # 横向抓
        elif "_2" in tag:
            return 1.47, 1.47  # 倒置抓
        else:
            return None  # 未知标签

    def target_info_callback(self, msg):
        if self.processing:
            self.get_logger().info("Manipulator is busy, skipping new target.")
            return

        try:
            # 解析格式：x:0.1 y:0.2 z:0.3 label:block_0
            data = dict(item.split(":") for item in msg.data.strip().split(" "))
            x = float(data['x'])
            y = float(data['y'])
            z = float(data['z'])
            label = data['label']
        except Exception as e:
            self.get_logger().error(f"Failed to parse message: {msg.data}")
            return

        # 容差比较判断是否为“新目标”
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
        tx, ty, tz = z + 0.13, x, -y - 0.02  # 坐标转换

        self.processing = True
        self.get_logger().info(f"Processing NEW target {label}: x={tx}, y={ty}, z={tz}, roll={roll}, pitch={pitch}")

        try:
            self.bot.gripper.release()
            self.bot.arm.set_ee_pose_components(x=tx, y=ty, z=tz, roll=roll, pitch=pitch)
            time.sleep(1)

            self.bot.gripper.grasp()
            time.sleep(1)

            self.bot.arm.go_to_home_pose()
            time.sleep(1)
            self.bot.arm.go_to_sleep_pose()
            self.bot.gripper.release()

            self.get_logger().info("Target processed successfully.")
            self.last_target = (x, y, z, label)  # 更新目标记录
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
