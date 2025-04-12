import rclpy
from rclpy.node import Node
import numpy as np
import open3d as o3d
import struct
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
import sensor_msgs_py.point_cloud2 as pc2


class CubeDetector(Node):
    def __init__(self):
        super().__init__('cube_detector')

        # Subscribe to the depth camera's PointCloud2 topic
        self.subscription = self.create_subscription(
            PointCloud2,
            '/depth/points',  # Change topic name if necessary
            self.process_pointcloud,
            10)
        
        # Publisher for the pose to approach the cube
        self.pose_publisher = self.create_publisher(PoseStamped, '/approach_pose', 10)

        self.get_logger().info("Cube Detector Node Initialized")

    def process_pointcloud(self, msg):
        """ Callback function to process incoming PointCloud2 data. """
        # Convert PointCloud2 to XYZ array
        points = self.pointcloud2_to_xyz(msg)

        if points is None or len(points) == 0:
            self.get_logger().warn("No valid points received.")
            return

        # Apply filtering to remove NaNs and out-of-range values
        points = points[~np.isnan(points).any(axis=1)]
        points = points[(points[:, 2] > 0.1) & (points[:, 2] < 2.0)]  # Depth range filter

        if len(points) < 500:
            self.get_logger().warn("Not enough points for processing.")
            return

        # Convert to Open3D point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        # Apply voxel downsampling
        pcd = pcd.voxel_down_sample(voxel_size=0.01)

        # Perform DBSCAN clustering
        labels = np.array(pcd.cluster_dbscan(eps=0.03, min_points=10, print_progress=False))


        if labels.size == 0 or np.max(labels) == -1:
            self.get_logger().warn("No clusters found.")
            return

        # Ensure points array matches the labels array size
        points = np.asarray(pcd.points)  # Extract updated point cloud from Open3D

        # Iterate through clusters and detect cube-like objects
        unique_clusters = np.unique(labels)
        for cluster in unique_clusters:
            if cluster == -1:
                continue  # Ignore noise points

            # Get indices of the current cluster
            cluster_indices = np.where(labels == cluster)[0]

            # Select only the cluster points
            cluster_pcd = pcd.select_by_index(cluster_indices)

            # Get bounding box of cluster
            bbox = cluster_pcd.get_axis_aligned_bounding_box()
    
            # Extract dimensions of the bounding box
            bbox_extent = bbox.get_extent()  # Returns (x, y, z) sizes

            # Check if bounding box dimensions match a cube (~20 cm sides)
            if all(0.15 <= dim <= 0.25 for dim in bbox_extent):
                self.get_logger().info(f"Detected Cube at {bbox.get_center()}")
    
                # Publish pose to approach the cube
                self.publish_approach_pose(bbox.get_center())
                return  # Exit after detecting the first cube

        self.get_logger().warn("No valid cubes detected.")
            
        
    def pointcloud2_to_xyz(self, cloud_msg):
        """ Convert PointCloud2 message to a NumPy array of XYZ points. """
        try:
            points = np.array([
                [p[0], p[1], p[2]] for p in pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True)
            ])
            return points
        except Exception as e:
            self.get_logger().error(f"Failed to convert PointCloud2: {e}")
            return None

                                          
def main(args=None):
    rclpy.init(args=args)
    node = CubeDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

