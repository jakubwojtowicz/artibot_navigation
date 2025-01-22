#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class MotionControllerNode(Node):
    def __init__(self):
        super().__init__('motion_controller_node')

        """
        Load configuration
        """
        self.declare_parameters(
            namespace='',
            parameters=[
                ('max_linear_speed', 0.5),
                ('min_linear_speed', 0.1),
                ('max_rotational_speed', 1.5),
                ('min_rotation_speed', 0.1),
                ('min_turning_radius', 0.2),
                ('controller_frequency', 10.0),
                ('linear_acceleration', 0.1),
                ('rotational_acceleration', 0.1),
            ])
        
        self.get_logger().info(f"Loaded configuration")

        # cmd_vel publisher
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(1.0/self.get_parameter('controller_frequency').get_parameter_value().double_value, self.publish_cmd_vel)
        # motion_command subscriber
        self.motion_command_sub = self.create_subscription(
            String, 'motion_command', self.motion_command_callback, 10
        )

        self.get_logger().info("Motion Controller Node initialized.")
    
    def publish_cmd_vel(self):
        # Tworzymy wiadomość typu Twist
        msg = Twist()

        # Ustawiamy wartości dla prędkości liniowej (x) i kątowej (z)
        msg.linear.x = 0.5  # Prędkość liniowa w osi x (do przodu)
        msg.angular.z = 0.1  # Prędkość kątowa wokół osi z (obroty w lewo)

        #Publikujemy wiadomość
        #self.publisher.publish(msg)
        self.get_logger().info(f"Publishing mock: linear.x = {msg.linear.x}, angular.z = {msg.angular.z}")

    def motion_command_callback(self, msg):
        """
        Motion command handler.
        """


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
