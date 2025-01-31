#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import math
import statistics

class ExplorationPlannerNode(Node):
    def __init__(self):
        super().__init__('exploration_planner_node')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('max_distance_to_turn', 2.0),
                ('motion_command_frequency', 10.0),
                ('inf_weight', 0.05),
                ('min_distance_weight', 1.0),
                ('side_angle_range', 45),
                ('max_distance_weight', 1.0)
            ])

        self.distance_treshold = self.get_parameter('max_distance_to_turn').get_parameter_value().double_value
        self.inf_weight = self.get_parameter('inf_weight').get_parameter_value().double_value
        self.min_distance_weight = self.get_parameter('min_distance_weight').get_parameter_value().double_value
        self.side_angle_range = self.get_parameter('side_angle_range').get_parameter_value().integer_value
        self.max_distance_weight = self.get_parameter('max_distance_weight').get_parameter_value().double_value

        # motion_controller publisher
        self.motion_controller_pub = self.create_publisher(String, 'motion_command', 10)
        self.timer = self.create_timer(1.0/self.get_parameter('motion_command_frequency').get_parameter_value().double_value, self.publish_motion_command)
        # nav_state subscriber
        self.nav_state_sub = self.create_subscription(
            String, 'nav_state', self.nav_state_callback, 10
        )
        # LaserScan subscriber
        self.laser_scan_sub = self.create_subscription(
            LaserScan, 'scan', self.laser_scan_callback, 10
        )
        
        self.nav_state = ''
        self.motion_command = ''
        self.get_logger().info("Exploration Planner node initialized.")

    def laser_scan_callback(self, msg):
        if self.nav_state not in ['collision', 'object_tracking', 'goal_achieved']:
            ranges = msg.ranges
            front_ranges = ranges[-15:] + ranges[:15]
            left_ranges = ranges[15:70]
            right_ranges = ranges[-70:-15]

            front_inf_count = len([value for value in front_ranges if math.isinf(value)])
            left_inf_count = len([value for value in left_ranges if math.isinf(value)])
            right_inf_count = len([value for value in right_ranges if math.isinf(value)])

            front_ranges_valid = [value for value in front_ranges if not math.isinf(value)]
            left_ranges_valid = [value for value in left_ranges if not math.isinf(value)]
            right_ranges_valid = [value for value in right_ranges if not math.isinf(value)]
           
            avg_front_ranges = 0.0
            avg_left_ranges = 0.0
            avg_right_ranges = 0.0

            if front_ranges_valid:
                avg_front_ranges = sum(front_ranges_valid) / len(front_ranges_valid)
            if left_ranges_valid:
                avg_left_ranges = sum(left_ranges_valid) / len(left_ranges_valid)
            if right_ranges_valid:
                avg_right_ranges = sum(right_ranges_valid) / len(right_ranges_valid)

            front_score = avg_front_ranges + self.inf_weight*front_inf_count - self.min_distance_weight/min(front_ranges) + self.max_distance_weight*max(front_ranges_valid)
            left_score = avg_left_ranges + self.inf_weight*left_inf_count - self.min_distance_weight/min(left_ranges) + self.max_distance_weight*max(left_ranges_valid)
            right_score = avg_right_ranges + self.inf_weight*right_inf_count - self.min_distance_weight/min(right_ranges) + self.max_distance_weight*max(right_ranges_valid)
            
            if (front_score>left_score) and (front_score>right_score) and (min(front_ranges) > self.distance_treshold):
                self.motion_command = 'forward'
            elif (left_score>front_score) and (left_score>right_score) and (min(left_ranges) > self.distance_treshold):
                self.motion_command = 'turn_left'
            elif (right_score>front_score) and (right_score>left_score) and (min(right_ranges) > self.distance_treshold):
                self.motion_command = 'turn_right'

    def publish_motion_command(self):
        if self.nav_state not in ['collision', 'object_tracking', 'goal_achieved']:
            msg = String()
            msg.data = self.motion_command 
            self.motion_controller_pub.publish(msg)
            #self.get_logger().info(f"Publishing {msg} motion command.")

    def nav_state_callback(self, msg):
        self.nav_state = msg.data

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
