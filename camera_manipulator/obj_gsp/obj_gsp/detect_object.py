import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
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

        # Align depth to the color image
        self.align = rs.align(rs.stream.color)

        # Set high-accuracy depth mode
        depth_sensor = profile.get_device().first_depth_sensor()
        depth_sensor.set_option(rs.option.visual_preset, 3)  # 3 = High-accuracy mode

        self.target_publisher = self.create_publisher(Point, 'target_position', 10)
        self.intrinsics = None

        # Get model path and load YOLO model
        package_name = "obj_gsp"
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
        """ Combine minimum depth and mean filtering for more reliable depth measurement (unit: m) """
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
        return min(min_depth, avg_depth)  # Return depth value in meters (m)

    def scan_and_output(self):
        while rclpy.ok():
            try:
                start_time = time.time()
                frames = self.pipeline.wait_for_frames()
                aligned_frames = self.align.process(frames)  # Align depth map and color image
                color_frame = aligned_frames.get_color_frame()
                depth_frame = aligned_frames.get_depth_frame()

                if not color_frame or not depth_frame:
                    self.get_logger().warning("No frames received from RealSense")
                    continue

                if self.intrinsics is None:
                    self.intrinsics = depth_frame.profile.as_video_stream_profile().get_intrinsics()

                color_image = np.asanyarray(color_frame.get_data())
                results = self.model(color_image)  # YOLOv8 Object Detection

                detected_objects = []
                for result in results:
                    for box in result.boxes:
                        x1, y1, x2, y2 = map(int, box.xyxy[0])  # Get bounding box
                        label = self.model.names[int(box.cls[0])]  # Get class name
                        confidence = float(box.conf[0])

                        # Calculate center point
                        center_x = (x1 + x2) // 2
                        center_y = (y1 + y2) // 2
                        w, h = x2 - x1, y2 - y1

                        # Obtain depth information (unit: m)
                        depth_value = self.get_valid_depth(depth_frame, center_x, center_y, w, h)
                        if depth_value == float('inf'):
                            continue

                        object_3d = self.pixel_to_point(center_x, center_y, depth_value)
                        object_3d = tuple(round(coord, 3) for coord in object_3d)  # Keep 3 decimal places
                        detected_objects.append((label, object_3d, confidence))

                        # Draw detection box
                        cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.putText(color_image, f"{label}: {object_3d} m", (x1, y1 - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # Select the target with the highest confidence for publishing
                if detected_objects:
                    best_object = max(detected_objects, key=lambda obj: obj[2])
                    self.publish_object(best_object)

                cv2.imshow("YOLO Object Detection", color_image)
                cv2.waitKey(1)

                # Control frame rate to avoid extremely low frame rates impacting performance
                elapsed_time = time.time() - start_time
                time.sleep(max(0, 1.0 / 30 - elapsed_time))  # Target frame rate: 30FPS

            except Exception as e:
                self.get_logger().error(f"Error during scanning: {e}")

    def publish_object(self, detected_object):
        label, position, confidence = detected_object
        point_msg = Point()
        point_msg.x, point_msg.y, point_msg.z = position
        self.target_publisher.publish(point_msg)
        self.get_logger().info(f"Published Object {label} at {position} m with confidence {confidence:.2f}")

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
