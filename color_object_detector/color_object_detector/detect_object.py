import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
import pyrealsense2 as rs
import cv2
import numpy as np
import time
import os
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge


class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        self.bridge = CvBridge()

        # 订阅图像和深度数据
        self.create_subscription(Image, '/camera/camera/color/image_raw', self.image_callback, 10)
        self.create_subscription(Image, '/camera/camera/depth/image_raw', self.depth_callback, 10)

        self.target_publisher = self.create_publisher(Point, 'target_position', 10)
        self.intrinsics = None

        # 获取模型路径并加载 YOLO 模型
        package_name = "color_object_detector"
        try:
            package_share_directory = get_package_share_directory(package_name)
            model_path = os.path.join(package_share_directory, "models", "best_copy.pt")

            if not os.path.exists(model_path):
                raise FileNotFoundError(f"Model file not found: {model_path}")

            self.model = YOLO(model_path)
            self.get_logger().info("YOLOv8 Object Detector Node Started")
        except Exception as e:
            self.get_logger().error(f"Error loading YOLO model: {e}")
            exit(1)

        self.latest_color_frame = None
        self.latest_depth_frame = None

    def image_callback(self, msg):
        """ 处理接收到的 RGB 图像 """
        self.latest_color_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.process_frame()

    def depth_callback(self, msg):
        """ 处理接收到的深度图像 """
        self.latest_depth_frame = self.bridge.imgmsg_to_cv2(msg, "16UC1")

    def get_valid_depth(self, x, y, w, h, kernel_size=5):
        """ 结合最小深度和均值滤波，获取更可靠的深度（单位：m） """
        if self.latest_depth_frame is None:
            return float('inf')

        half_k = kernel_size // 2
        depth_values = []
        min_depth = float('inf')

        for i in range(-half_k, half_k + 1):
            for j in range(-half_k, half_k + 1):
                d = self.latest_depth_frame[y + j, x + i] * 0.001  # 转换为米
                if 0 < d < min_depth:
                    min_depth = d
                if d > 0:
                    depth_values.append(d)

        if len(depth_values) == 0:
            return float('inf')
        avg_depth = np.mean(depth_values)
        return min(min_depth, avg_depth)

    def process_frame(self):
        if self.latest_color_frame is None or self.latest_depth_frame is None:
            return

        color_image = self.latest_color_frame
        results = self.model(color_image)  # YOLOv8 目标检测

        detected_objects = []
        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])  # 获取边界框
                label = self.model.names[int(box.cls[0])]  # 获取类别名称
                confidence = float(box.conf[0])

                # 计算中心点
                center_x = (x1 + x2) // 2
                center_y = (y1 + y2) // 2
                w, h = x2 - x1, y2 - y1

                # 获取深度信息（单位：m）
                depth_value = self.get_valid_depth(center_x, center_y, w, h)
                if depth_value == float('inf'):
                    continue

                object_3d = self.pixel_to_point(center_x, center_y, depth_value)
                object_3d = tuple(round(coord, 3) for coord in object_3d)  # 保留3位小数
                detected_objects.append((label, object_3d, confidence))

                # 绘制检测框
                cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(color_image, f"{label}: {object_3d} m", (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # 选择置信度最高的目标进行发布
        if detected_objects:
            best_object = max(detected_objects, key=lambda obj: obj[2])
            self.publish_object(best_object)

        cv2.imshow("YOLO Object Detection", color_image)
        cv2.waitKey(1)

    def publish_object(self, detected_object):
        label, position, confidence = detected_object
        point_msg = Point()
        point_msg.x, point_msg.y, point_msg.z = position
        self.target_publisher.publish(point_msg)
        self.get_logger().info(f"Published Object {label} at {position} m with confidence {confidence:.2f}")

    def pixel_to_point(self, pixel_x, pixel_y, depth_value):
        return tuple(rs.rs2_deproject_pixel_to_point(self.intrinsics, [pixel_x, pixel_y], depth_value))

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
