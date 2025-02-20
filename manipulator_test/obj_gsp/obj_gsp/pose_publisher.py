import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from obj_gsp.object_detection import ObjectDetection
import cv2
from collections import deque
import numpy as np


class DesiredPositionPublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')
        # Correct the attribute name to match the initialization
        self.publisher = self.create_publisher(
            Pose,
            '/pose_topic',
            1)  # Using 1 for the QoS profile for simplicity

        # Immediately send the desired position once after creation
        self.publish_desired_position()

    def publish_desired_position(self):
        # Correct attribute reference to 'self.publisher'
        while self.publisher.get_subscription_count() == 0:
            self.get_logger().info('Waiting for subscribers...')
            # Use a small timeout for spin_once to allow other operations to proceed
            rclpy.spin_once(self, timeout_sec=0.5)

        msg = Pose()
        # msg.position = Point(x=0.3, y=0.0, z=0.055)
        ###########################################################
        obj = ObjectDetection()
        coordinates = deque(maxlen=50)
        while True:
            coordinate = obj.main()
            if coordinate is not None:
                coordinates.append(coordinate)
                if len(coordinates) == 50:
                    avg_coordinate = np.mean(np.array(coordinates), axis=0)
                    print('Average coordinate: ', avg_coordinate)
                    break

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        obj.pipeline.stop()
        cv2.destroyAllWindows()

        msg.position = Point(x=avg_coordinate[0], y=avg_coordinate[1], z=avg_coordinate[2])
        ###########################################################
        msg.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        # Publishing the message
        self.publisher.publish(msg)
        self.get_logger().info(f"Sent desired position: x={msg.position.x}, y={msg.position.y}, z={msg.position.z}")


def main(args=None):
    rclpy.init(args=args)
    node = DesiredPositionPublisher()
    rclpy.spin_once(node, timeout_sec=1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()