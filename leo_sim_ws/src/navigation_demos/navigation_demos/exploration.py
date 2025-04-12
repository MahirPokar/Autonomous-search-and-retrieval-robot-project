#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

# For dummy subscriptions or additional info
from std_msgs.msg import String
from geometry_msgs.msg import Twist

# Optional for publishing frontier markers or debugging
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

import numpy as np


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('exploration')
        
        # === Subscriptions ===
        # 1) Subscribe to /map
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        

    def map_callback(self, msg: OccupancyGrid):
        """
        Callback each time a new map is received on /map.
        We'll parse the map to find a naive frontier, then
        send a single goal to navigate there.
        """

        self.get_logger().info('Received a new map!')

        # Convert OccupancyGrid data to a numpy array for easier processing
        map_data = np.array(msg.data, dtype=np.int8)
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution

        # Reshape the map into 2D
        # OccupancyGrid is in row-major format: 
        #   index = y * width + x
        # So shape will be (height, width).
        map_2d = np.reshape(map_data, (height, width))

        # Find frontier cells
        frontier_cells = self.find_frontiers(map_2d)

        # If no frontiers found, do nothing (or handle logic differently)
        if not frontier_cells:
            self.get_logger().info('No frontier found. Exploration complete or map fully known.')
            return

        # Choose the first frontier cell (very naive)
        chosen_frontier = frontier_cells[0]

        # Convert the cell coordinate (row=y, col=x) to a pose in the map frame
        frontier_pose = self.cell_to_pose(chosen_frontier, msg.info)

        # (Optional) Publish frontier markers for visualization
        self.publish_frontier_markers(frontier_cells, msg.info)

        # Send the chosen goal to Nav2
        self.send_goal(frontier_pose)

    def find_frontiers(self, map_2d: np.ndarray):
        """
        Naive frontier detection:
          - A cell is considered unknown (value == -1).
          - A 'frontier cell' is unknown AND has at least one free (0) neighbor.
        This is a simplistic approach; many real frontier detectors are more elaborate.
        """

        frontiers = []
        height, width = map_2d.shape
        
        # Directions for neighbors (8-connected, or you can use 4-connected)
        neighbors_8 = [(-1, -1), (-1, 0), (-1, 1),
                       ( 0, -1),          ( 0, 1),
                       ( 1, -1), ( 1, 0), ( 1, 1)]

        for r in range(height):
            for c in range(width):
                if map_2d[r, c] == -1:
                    # Check neighbors for free space
                    for nr, nc in neighbors_8:
                        rr = r + nr
                        cc = c + nc
                        if (0 <= rr < height) and (0 <= cc < width):
                            if map_2d[rr, cc] == 0:
                                # It's a frontier cell
                                frontiers.append((r, c))
                                break
        return frontiers

    def cell_to_pose(self, cell, info):
        """
        Convert a (row, col) = (y, x) cell index into a PoseStamped in the 'map' frame.
        The origin of the map is typically in the lower-left corner (depending on your setup).
        We'll assume a simple transform: 
          map_x = origin_x + (col + 0.5) * resolution
          map_y = origin_y + (row + 0.5) * resolution
        """
        r, c = cell
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y
        resolution = info.resolution

        map_x = origin_x + (c + 0.5) * resolution
        map_y = origin_y + (r + 0.5) * resolution

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = map_x
        pose.pose.position.y = map_y
        pose.pose.orientation.w = 1.0  # No rotation

        return pose

    def publish_frontier_markers(self, frontier_cells, info):
        """
        Publish a MarkerArray for visualizing frontier cells in RViz or similar.
        """
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        for i, cell in enumerate(frontier_cells):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = now
            marker.ns = 'frontiers'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            # Convert cell to map coords
            pose = self.cell_to_pose(cell, info)
            marker.pose = pose.pose
            
            # Size
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            
            # Color (e.g. green)
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0

            marker_array.markers.append(marker)

        self.frontier_publisher.publish(marker_array)

    def send_goal(self, pose_stamped: PoseStamped):
        """
        Sends a NavigateToPose action goal to the Nav2 action server.
        """
        # Wait until action server is available
        while not self._nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for NavigateToPose action server...')
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_stamped
        
        # Optionally set a Behavior Tree name if you want:
        # goal_msg.behavior_tree = '...'  # leave empty if not used

        self.get_logger().info(
            f'Sending goal to frontier at ({pose_stamped.pose.position.x:.2f}, '
            f'{pose_stamped.pose.position.y:.2f})'
        )

        send_goal_future = self._nav_to_pose_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """
        Callback for the result of sending the goal.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal was rejected by the server.')
            return

        self.get_logger().info('Goal accepted! Waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """
        Callback for when the goal is done executing.
        """
        self.get_logger().info(f'Goal result received: {result}')
        

def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

