#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import time

class CollisionAvoidancePlannerNode(Node):
    def __init__(self):
        super().__init__('collision_avoidance_planner_node')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('distance_from_obstacle_treshold', 0.2),
                ('backtracking_enabled', True),
                ('backtracking_max_time', 3),
                ('motion_command_frequency', 5.0),
                ('sleep_after_collision_time', 3)
            ])
        
        self.distance_from_obstacle_treshold = self.get_parameter('distance_from_obstacle_treshold').get_parameter_value().double_value
        self.backtracking_enabled = self.get_parameter('backtracking_enabled').get_parameter_value().bool_value
        self.backtracking_max_time = self.get_parameter('backtracking_max_time').get_parameter_value().integer_value
        self.sleep_after_collision_time = self.get_parameter('sleep_after_collision_time').get_parameter_value().integer_value
        self.backtracking = False
        self.backtracikng_start_time = time.time()
        self.nav_state = ''
        # nav_state publisher
        self.nav_state_pub = self.create_publisher(String, 'nav_state', 10)
        # motion_controller publisher
        self.motion_controller_pub = self.create_publisher(String, 'motion_command', 10)
        # LaserScan subscriber
        self.laser_scan_sub = self.create_subscription(
            LaserScan, 'scan', self.laser_scan_callback, 10
        )
        # nav_state subscriber
        self.nav_state_sub = self.create_subscription(
            String, 'nav_state', self.nav_state_callback, 10
        )

        self.get_logger().info("Collision Avoidance Planner node initialized.")

    def laser_scan_callback(self, msg):
        if self.nav_state != 'object_tracking' and self.nav_state != 'goal_achieved':
            ranges = msg.ranges
            front_ranges = ranges[-15:] + ranges[:15]
            if self.backtracking:
                back_ranges = ranges[165:195]
                if(min(back_ranges) <= self.distance_from_obstacle_treshold*2.0):
                    self.finish_backtracking()
            elif min(front_ranges) < self.distance_from_obstacle_treshold:
                self.get_logger().info("Collision detected. Setting collision state and stopping the robot.")
                nav_state = String()
                nav_state.data = 'collision'
                self.nav_state_pub.publish(nav_state)
                motion_command = String()
                motion_command.data = 'stop'
                self.motion_controller_pub.publish(motion_command)
                time.sleep(self.sleep_after_collision_time)
                if self.backtracking_enabled:
                    self.backtracikng_start_time = time.time()
                    self.backtracking = True
                    self.get_logger().info("Moving back...")
                    self.timer = self.create_timer(1.0/self.get_parameter('motion_command_frequency').get_parameter_value().double_value, self.robot_go_back)

    def robot_go_back(self):
        if time.time() - self.backtracikng_start_time > self.backtracking_max_time:
            self.finish_backtracking()
        else:
            msg = String()
            msg.data = 'backward'
            self.motion_controller_pub.publish(msg)
    
    def finish_backtracking(self):
        self.get_logger().info("Exiting collsion state.")
        self.backtracking = False
        self.timer.destroy()
        motion_command = String()
        motion_command.data = 'stop'
        self.motion_controller_pub.publish(motion_command)
        nav_state = String()
        nav_state.data = 'exploration'
        self.nav_state_pub.publish(nav_state)

    def nav_state_callback(self, msg):
        self.nav_state = msg.data


def main(args=None):
    rclpy.init(args=args)
    node = CollisionAvoidancePlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
