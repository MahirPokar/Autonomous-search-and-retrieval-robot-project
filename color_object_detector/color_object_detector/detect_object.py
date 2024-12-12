import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
from collections import defaultdict


class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')

        # 创建发布器
        self.target_publisher = self.create_publisher(Point, 'target_position', 10)

        # 订阅相机的话题
        self.color_subscription = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.color_callback, 10)
        self.depth_subscription = self.create_subscription(
            Image, '/camera/camera/depth/image_rect_raw', self.depth_callback, 10)

        self.bridge = CvBridge()  # 用于ROS图像消息和OpenCV图像转换
        self.color_image = None
        self.depth_image = None

        self.color_ranges = {
            'red': [
                (np.array([0, 150, 50]), np.array([5, 255, 255])),
                (np.array([175, 150, 50]), np.array([180, 255, 255]))
            ],
            'orange': [
                (np.array([5, 150, 50]), np.array([25, 255, 255]))
            ],
            'yellow': [
                (np.array([25, 150, 150]), np.array([35, 255, 255]))
            ],
            'green': [
                (np.array([35, 100, 50]), np.array([85, 255, 255]))
            ],
            'blue': [
                (np.array([100, 150, 50]), np.array([130, 255, 255]))
            ],
            'pink': [
                (np.array([140, 20, 20]), np.array([180, 255, 255]))
            ],
            'purple': [
                (np.array([120, 20, 20]), np.array([170, 255, 255]))
            ]
        }

        self.start_time = time.time()
        self.object_count = defaultdict(int)
        self.object_closest = defaultdict(lambda: float('inf'))
        self.object_positions = {}
        self.get_logger().info("Object Detector Node Started")

    def color_callback(self, msg):
        self.color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")

    def process_frames(self):
        if self.color_image is None or self.depth_image is None:
            self.get_logger().info("Waiting for color and depth frames...")
            return

        hsv = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2HSV)

        for color_name, ranges in self.color_ranges.items():
            mask = None
            for lower, upper in ranges:
                color_mask = cv2.inRange(hsv, lower, upper)
                mask = color_mask if mask is None else mask + color_mask

            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            contours = [c for c in contours if cv2.contourArea(c) > 500]

            for c in contours:
                x, y, w, h = cv2.boundingRect(c)
                center_x = x + w // 2
                center_y = y + h // 2

                if center_x < 0 or center_y < 0 or center_x >= self.depth_image.shape[1] or center_y >= self.depth_image.shape[0]:
                    continue

                depth_value = self.depth_image[center_y, center_x]

                if depth_value > 0:
                    object_3d = self.pixel_to_point(center_x, center_y, depth_value)
                    object_3d = tuple(round(coord, 2) for coord in object_3d)

                    self.object_count[color_name] += 1
                    if depth_value < self.object_closest[color_name]:
                        self.object_closest[color_name] = depth_value
                        self.object_positions[color_name] = object_3d

                    cv2.rectangle(self.color_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.circle(self.color_image, (center_x, center_y), 5, (255, 0, 0), -1)
                    cv2.putText(self.color_image, f"{color_name}: {object_3d}",
                                (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        cv2.imshow("Object Detection", self.color_image)
        cv2.waitKey(1)

        # 每秒处理一次统计数据
        if time.time() - self.start_time >= 1.0:
            self.publish_most_detected_object()
            self.start_time = time.time()
            self.object_count.clear()
            self.object_closest.clear()
            self.object_positions.clear()

    def publish_most_detected_object(self):
        if not self.object_count:
            self.get_logger().info("No objects detected in the last second.")
            return

        # 找到识别次数最多的物体
        most_detected_color = max(self.object_count, key=self.object_count.get)
        most_detected_count = self.object_count[most_detected_color]

        # 检查是否有其他颜色的识别次数接近（差异小于30%）
        close_colors = [
            color for color, count in self.object_count.items()
            if abs(count - most_detected_count) / most_detected_count <= 0.3
        ]

        # 如果有多个接近的颜色，选择最近的物体
        if len(close_colors) > 1:
            closest_color = min(close_colors, key=lambda color: self.object_closest[color])
            selected_color = closest_color
        else:
            selected_color = most_detected_color

        # 发布选定的物体位置
        selected_position = self.object_positions[selected_color]
        point_msg = Point()
        point_msg.x = float(selected_position[0])
        point_msg.y = float(selected_position[1])
        point_msg.z = float(selected_position[2])
        self.target_publisher.publish(point_msg)

        self.get_logger().info(
            f"Published {selected_color} object at position {selected_position} "
            f"with count {self.object_count[selected_color]} and closest distance {self.object_closest[selected_color]:.2f}"
        )

    def pixel_to_point(self, pixel_x, pixel_y, depth_value):
        fx = 615.0  # 假设的相机内参
        fy = 615.0
        cx = 320.0
        cy = 240.0
        x = (pixel_x - cx) * depth_value / fx
        y = (pixel_y - cy) * depth_value / fy
        z = depth_value
        return x, y, z


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    try:
        while rclpy.ok():
            rclpy.spin_once(node)
            node.process_frames()
    except KeyboardInterrupt:
        print("Shutting down node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
