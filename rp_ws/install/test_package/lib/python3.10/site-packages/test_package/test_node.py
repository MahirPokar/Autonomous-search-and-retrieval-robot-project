import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

class ClosestDistancePublisher(Node):
    def __init__(self):
        super().__init__('closest_distance_publisher')
        
        # Create a subscriber to listen to the /scan topic (from RPLIDAR)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10  # QoS profile
        )
        
        # Create a publisher to publish the closest distance on another topic
        self.publisher = self.create_publisher(Float32, '/closest_distance', 10)

    def scan_callback(self, msg: LaserScan):
        # Find the closest distance from the scan data
        # LaserScan ranges are in msg.ranges, where each element represents a distance at that angle
        closest_distance = min([distance for distance in msg.ranges if distance > 0])  # Filter out 'inf' and 'NaN'
        
        # Create a message to publish
        closest_distance_msg = Float32()
        closest_distance_msg.data = closest_distance
        
        # Publish the closest distance
        self.publisher.publish(closest_distance_msg)
        self.get_logger().info(f'Closest Distance: {closest_distance} meters')

def main(args=None):
    rclpy.init(args=args)
    node = ClosestDistancePublisher()
    rclpy.spin(node)

    # Shutdown cleanly
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

