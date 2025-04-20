import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
from ultralytics import YOLO
import message_filters
import numpy as np
import cv2
import math
import os

from ament_index_python.packages import get_package_share_directory
from tf2_ros import Buffer, TransformListener
from tf_transformations import quaternion_from_euler
from tf2_geometry_msgs import do_transform_pose
# Custom message
from obj_gsp_interfaces.msg import BlockGoalList  # Adjust this import to your actual package name
from obj_gsp_interfaces.msg import BlockGoal
from obj_gsp_interfaces.srv import GetBlockGoals

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('Object_detection_with_service')

        self.bridge = CvBridge()
        self.intrinsics = None
        self.goal_list = []
        self.label_list = []
        self.current_robot_pose = None

        # TF setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Load YOLO model
        try:
            package_path = get_package_share_directory("obj_gsp")
            model_path = os.path.join(package_path, "models", "best_copy.pt")
            if not os.path.exists(model_path):
                raise FileNotFoundError(f"Model not found at {model_path}")
            self.model = YOLO(model_path)
            self.get_logger().info("✅ YOLO model loaded")
        except Exception as e:
            self.get_logger().error(f"❌ YOLO load failed: {e}")
            exit(1)

        # Subscriptions
        self.color_sub = message_filters.Subscriber(self, Image, '/camera/d435i_camera/color/image_raw')
        # self.depth_sub = message_filters.Subscriber(self, Image, '/camera/d435i_camera/depth/image_rect_raw')
        self.depth_sub = message_filters.Subscriber(self, Image, '/camera/d435i_camera/aligned_depth_to_color/image_raw')
        self.info_sub  = message_filters.Subscriber(self, CameraInfo, '/camera/d435i_camera/color/camera_info')
        # self.ts = message_filters.ApproximateTimeSynchronizer(
        #     [self.color_sub, self.depth_sub, self.info_sub], queue_size=10, slop=0.1)
        self.ts = message_filters.ApproximateTimeSynchronizer(
    [self.color_sub, self.depth_sub, self.info_sub], queue_size=10, slop=0.05)

        self.ts.registerCallback(self.image_callback)

        # Publishers
        self.block_goals = []  # List of BlockGoal
        self.get_block_goals_srv = self.create_service(GetBlockGoals, 'get_block_goals', self.handle_get_block_goals)
        self.target_publisher = self.create_publisher(String, 'target_publisher', 10)
    
        self.get_logger().info("🚀 Object Detector Node is ready.")

        # --- Inject fake block manually ---
        fake_block = BlockGoal()
        fake_block.pose.header.frame_id = 'map'
        fake_block.pose.header.stamp = self.get_clock().now().to_msg()
        fake_block.pose.pose.position.x = 0.2
        fake_block.pose.pose.position.y = 7.0
        fake_block.pose.pose.position.z = 0.0
        fake_block.pose.pose.orientation.w = 1.0
        fake_block.label = "yellow_0"  # or any valid label

        self.block_goals.append(fake_block)
        self.get_logger().info("🧪 Injected fake block at (0.64, 2.82)")
        # --- done ---

    def image_callback(self, color_msg, depth_msg, info_msg):
        try:
            color_image = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

            if self.intrinsics is None:
                self.intrinsics = {
                    "fx": info_msg.k[0],
                    "fy": info_msg.k[4],
                    "cx": info_msg.k[2],
                    "cy": info_msg.k[5]
                }
                self.get_logger().info(f"🎯 Camera intrinsics loaded: {self.intrinsics}")

            results = self.model(color_image)
            detected_objects = []

            for result in results:
                for box in result.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    label = self.model.names[int(box.cls[0])]
                    cx = (x1 + x2) // 2
                    cy = (y1 + y2) // 2

                    depth = self.get_valid_depth(depth_image, cx, cy)
                    if np.isinf(depth):
                        continue

                    position = self.pixel_to_point(cx, cy, depth)
                    detected_objects.append((label, position))

                    cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(color_image, f"{label}: {position}", (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            if detected_objects:
                self.handle_detected_objects(detected_objects)
                self.publish_object(detected_objects[0])

            cv2.imshow("YOLO Detections", color_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"❌ image_callback error: {e}")

    def get_valid_depth(self, depth_image, x, y, kernel_size=5):
        h, w = depth_image.shape
        x_min = max(x - kernel_size // 2, 0)
        x_max = min(x + kernel_size // 2 + 1, w)
        y_min = max(y - kernel_size // 2, 0)
        y_max = min(y + kernel_size // 2 + 1, h)
        roi = depth_image[y_min:y_max, x_min:x_max]
        valid = roi[roi > 0]
        return np.mean(valid) * 0.001 if valid.size > 0 else float('inf')

    def pixel_to_point(self, x, y, z):
        fx = self.intrinsics["fx"]
        fy = self.intrinsics["fy"]
        cx = self.intrinsics["cx"]
        cy = self.intrinsics["cy"]
        X = (x - cx) * z / fx
        Y = (y - cy) * z / fy
        Z = z
        return round(X, 3), round(Y, 3), round(Z, 3)

    def build_camera_pose(self, x, y, z):
        pose = PoseStamped()
        pose.header.stamp = rclpy.time.Time().to_msg()
        pose.header.frame_id = 'd435i_camera_link'
        pose.pose.position.x = z
        pose.pose.position.y = x
        pose.pose.position.z = y
        pose.pose.orientation.w = 1.0
        return pose

    def handle_detected_objects(self, objects):
        for label, position in objects:
            pose_cam = self.build_camera_pose(*position)

            try:
                pose_map = self.tf_buffer.transform(pose_cam, 'map', timeout=rclpy.duration.Duration(seconds=1.0))
            except Exception as e:
                self.get_logger().error(f"❌ TF transform to 'map' failed: {e}")
                continue

            if pose_map:
                is_duplicate = any(
                        math.dist(
                            [bg.pose.pose.position.x, bg.pose.pose.position.y],
                            [pose_map.pose.position.x, pose_map.pose.position.y]
                        ) < 0.5 for bg in self.block_goals
                    )

                if not is_duplicate:
                    block_goal = BlockGoal()
                    block_goal.pose = pose_map
                    block_goal.label = label
                    self.block_goals.append(block_goal)
                    self.get_logger().info(f"🟢 Goal added for '{label}'")

    def handle_get_block_goals(self, request, response):
        response.goals = self.block_goals
        self.get_logger().info(f"📨 Sent {len(self.block_goals)} goals to FSM node.")
        return response
    
    def publish_object(self, detected_object):
        label, position = detected_object
        object_msg = String()
        object_msg.data = f"x:{position[0]} y:{position[1]} z:{position[2]} label:{label}"

        self.target_publisher.publish(object_msg)
        self.get_logger().info(f"Published: {object_msg.data}")

        

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("👋 Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
