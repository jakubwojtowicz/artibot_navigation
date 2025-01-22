#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from artibot_object_detection.msg import BoundingBox # type: ignore

class ObjectTrackingPlannerNode(Node):
    def __init__(self):
        super().__init__('object_tracking_planner_node')

        """
        Load configuration
        """
        self.declare_parameters(
            namespace='',
            parameters=[
                ('min_detected_score', 0.4)
            ])
        
        self.get_logger().info(f"Loaded configuration")

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
        self.detected_object_sub = self.create_subscription(
            BoundingBox, 'detected_object', self.detected_object_callback, 10
        )

        self.get_logger().info("Object tracking planner node initialized.")

    def detected_object_callback(self, msg):
        """
        Detected object data handler.
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
    node = ObjectTrackingPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
