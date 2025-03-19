import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String
import pyrealsense2 as rs
import cv2
import numpy as np
import time
import os
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory


class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
        profile = self.pipeline.start(config)

        # 对齐深度到彩色图像
        self.align = rs.align(rs.stream.color)

        # 设置高精度深度模式
        depth_sensor = profile.get_device().first_depth_sensor()
        depth_sensor.set_option(rs.option.visual_preset, 3)  # 3 = 高精度模式

        self.target_publisher = self.create_publisher(String, 'target_position', 10)
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

    def get_valid_depth(self, depth_frame, x, y, w, h, kernel_size=5):
        """ 结合最小深度和均值滤波，获取更可靠的深度（单位：m） """
        half_k = kernel_size // 2
        depth_values = []
        min_depth = float('inf')

        for i in range(-half_k, half_k + 1):
            for j in range(-half_k, half_k + 1):
                d = depth_frame.get_distance(x + i, y + j)
                if 0 < d < min_depth:
                    min_depth = d
                if d > 0:
                    depth_values.append(d)

        if len(depth_values) == 0:
            return float('inf')
        avg_depth = np.mean(depth_values)
        return min(min_depth, avg_depth)  # 以米(m)为单位返回深度值

    def scan_and_output(self):
        while rclpy.ok():
            try:
                start_time = time.time()
                frames = self.pipeline.wait_for_frames()
                aligned_frames = self.align.process(frames)  # 对齐深度图和彩色图
                color_frame = aligned_frames.get_color_frame()
                depth_frame = aligned_frames.get_depth_frame()

                if not color_frame or not depth_frame:
                    self.get_logger().warning("No frames received from RealSense")
                    continue

                if self.intrinsics is None:
                    self.intrinsics = depth_frame.profile.as_video_stream_profile().get_intrinsics()

                color_image = np.asanyarray(color_frame.get_data())
                results = self.model(color_image)  # YOLOv8 目标检测

                detected_objects = []
                for result in results:
                    for box in result.boxes:
                        x1, y1, x2, y2 = map(int, box.xyxy[0])  # 获取边界框
                        label = self.model.names[int(box.cls[0])]  # 获取类别名称

                        # 计算中心点
                        center_x = (x1 + x2) // 2
                        center_y = (y1 + y2) // 2
                        w, h = x2 - x1, y2 - y1

                        # 获取深度信息（单位：m）
                        depth_value = self.get_valid_depth(depth_frame, center_x, center_y, w, h)
                        if depth_value == float('inf'):
                            continue

                        object_3d = self.pixel_to_point(center_x, center_y, depth_value)
                        object_3d = tuple(round(coord, 3) for coord in object_3d)  # 保留3位小数
                        detected_objects.append((label, object_3d))

                        # 绘制检测框
                        cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.putText(color_image, f"{label}: {object_3d} m", (x1, y1 - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # 选择检测到的目标进行发布
                if detected_objects:
                    best_object = detected_objects[0]
                    self.publish_object(best_object)

                cv2.imshow("YOLO Object Detection", color_image)
                cv2.waitKey(1)

                # 控制帧率，防止过低帧率影响体验
                elapsed_time = time.time() - start_time
                time.sleep(max(0, 1.0 / 30 - elapsed_time))  # 目标帧率30FPS

            except Exception as e:
                self.get_logger().error(f"Error during scanning: {e}")

    def publish_object(self, detected_object):
        label, position = detected_object
        object_msg = String()
        object_msg.data = f"x:{position[0]} y:{position[1]} z:{position[2]} label:{label}"

        self.target_publisher.publish(object_msg)
        self.get_logger().info(f"Published: {object_msg.data}")

    def pixel_to_point(self, pixel_x, pixel_y, depth_value):
        return tuple(rs.rs2_deproject_pixel_to_point(self.intrinsics, [pixel_x, pixel_y], depth_value))

    def destroy_node(self):
        self.pipeline.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    try:
        node.scan_and_output()
    except KeyboardInterrupt:
        print("Shutting down node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
