#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class MotionControllerNode(Node):
    def __init__(self):
        super().__init__('motion_controller_node')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('max_linear_speed', 0.3),
                ('max_linear_speed_object_tracking', 0.2),
                ('min_linear_speed', -0.3),
                ('max_rotation_speed', 1.5),
                ('min_rotation_speed', -1.5),
                ('min_turning_radius', 0.2),
                ('controller_frequency', 10.0),
                ('rotational_acceleration', 0.1),
                ('rotational_acceleration_object_tracking', 0.1)
            ])
        
        self.max_linear_vel = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        self.min_linear_vel = self.get_parameter('min_linear_speed').get_parameter_value().double_value
        self.rot_accel = self.get_parameter('rotational_acceleration').get_parameter_value().double_value
        self.max_rot_speed = self.get_parameter('max_rotation_speed').get_parameter_value().double_value
        self.min_rot_speed = self.get_parameter('min_rotation_speed').get_parameter_value().double_value

        self.linear_x = 0.0
        self.angular_z = 0.0
        self.nav_state = ''

        # cmd_vel publisher
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(1.0/self.get_parameter('controller_frequency').get_parameter_value().double_value, self.publish_cmd_vel)
        # motion_command subscriber
        self.motion_command_sub = self.create_subscription(
            String, 'motion_command', self.motion_command_callback, 10
        )
        # nav_state subscriber
        self.nav_state_sub = self.create_subscription(
            String, 'nav_state', self.nav_state_callback, 10
        )

        self.timer_linear_vel = self.create_timer(1, self.linear_speed_controller)
        self.timer_angular_vel = self.create_timer(0.1, self.angular_speed_controller)
        self.current_command = ''

        self.get_logger().info("Motion Controller Node initialized.")
    
    def publish_cmd_vel(self):
        msg = Twist()
        msg.linear.x = self.linear_x 
        msg.angular.z = self.angular_z 
        self.cmd_vel_pub.publish(msg)
        #self.get_logger().info(f"Publishing: linear.x = {msg.linear.x}, angular.z = {msg.angular.z}.")

    def motion_command_callback(self, msg):
        #self.get_logger().info(f"Received command message: {msg}.")
        if msg.data == 'forward':
            self.current_command = 'forward'
        elif msg.data == 'backward':
            self.current_command = 'backward'
        elif msg.data == 'turn_right':
            self.current_command = 'turn_right'
        elif msg.data == 'turn_left':
            self.current_command = 'turn_left'
        elif msg.data == 'stop':
            self.current_command = 'stop'

    def linear_speed_controller(self):
        if self.current_command == 'forward':
            self.linear_x = self.max_linear_vel
        elif self.current_command == 'backward':
            self.linear_x = self.min_linear_vel
        elif self.current_command == 'stop':
            self.linear_x = 0.0
            self.angular_z = 0.0

    def angular_speed_controller(self):
        if self.current_command == 'turn_right':
            if self.angular_z > self.min_rot_speed:
                self.linear_x = self.max_linear_vel
                self.angular_z -= self.rot_accel
        elif self.current_command == 'turn_left':
            if self.angular_z < self.max_rot_speed:
                self.linear_x = self.max_linear_vel
                self.angular_z += self.rot_accel
        elif self.current_command == 'forward' or self.current_command == 'backward':
            if self.angular_z < 0:
                self.angular_z += self.rot_accel
            elif self.angular_z > 0:
                self.angular_z -= self.rot_accel

    def nav_state_callback(self, msg):
        self.nav_state = msg.data
        if msg.data == 'object_tracking':
            self.max_linear_vel = self.get_parameter('max_linear_speed_object_tracking').get_parameter_value().double_value
            self.rot_accel = self.get_parameter('rotational_acceleration_object_tracking').get_parameter_value().double_value
        else:
            self.max_linear_vel = self.get_parameter('max_linear_speed').get_parameter_value().double_value
            self.rot_accel = self.get_parameter('rotational_acceleration').get_parameter_value().double_value

def main(args=None):
    rclpy.init(args=args)
    node = MotionControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
