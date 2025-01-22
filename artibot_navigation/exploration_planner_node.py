#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class ExplorationPlannerNode(Node):
    def __init__(self):
        super().__init__('exploration_planner_node')

        """
        Load configuration
        """
        self.declare_parameters(
            namespace='',
            parameters=[
                ('max_distance_to_turn', 2.0)
            ])

        self.get_logger().info(f"Loaded configuration.")

        """
        Set publishers and subsribers
        """
        # nav_state publisher
        self.nav_state_pub = self.create_publisher(String, 'nav_state', 10)
        # motion_controller publisher
        self.motion_controller_pub = self.create_publisher(String, 'motion_command', 10)
        # nav_state subscriber
        self.nav_state_sub = self.create_subscription(
            String, 'nav_state', self.nav_state_callback, 10
        )
        # LaserScan subscriber
        self.laser_scan_sub = self.create_subscription(
            LaserScan, 'scan', self.laser_scan_callback, 10
        )

        self.get_logger().info("Exploration Planner node initialized.")

    def laser_scan_callback(self, msg):
        """
        LIDAR data handler.
        """
        self.obstacle_detected = any(
            range < self.obstacle_distance_threshold for range in msg.ranges
        )

    def nav_state_callback(self, msg):
        """
        Navigation state handler.
        """

def main(args=None):
    rclpy.init(args=args)
    node = ExplorationPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
