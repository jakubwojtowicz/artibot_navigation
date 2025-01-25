#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from artibot_object_detection.msg import BoundingBox # type: ignore

class ObjectTrackingPlannerNode(Node):
    def __init__(self):
        super().__init__('object_tracking_planner_node')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('object_name', 'nike_ball'),
                ('not_found_value', 'not_found'),
                ('motion_command_frequency', 5.0),
                ('image_height', 480.0),
                ('image_width', 640.0),
            ])
        
        self.object_name = self.get_parameter('object_name').get_parameter_value().string_value
        self.not_found_value = self.get_parameter('not_found_value').get_parameter_value().string_value
        self.motion_command_frequency = self.get_parameter('motion_command_frequency').get_parameter_value().double_value
        self.image_height = self.get_parameter('image_height').get_parameter_value().double_value
        self.image_width = self.get_parameter('image_width').get_parameter_value().double_value
        self.nav_state = ''
        self.motion_command = ''
        self.object_tracking = False
        
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
            BoundingBox, 'detected_object_bounding_box', self.detected_object_callback, 10
        )

        self.get_logger().info("Object tracking planner node initialized.")

    def detected_object_callback(self, msg):
        if  self.object_tracking == False and self.nav_state != 'collision' and msg.label == self.object_name:
            #when object is found -> go to object tracking state
            self.object_tracking = True
            self.get_logger().info("Object found. Starting tracking the object.")
            nav_state = String()
            nav_state.data = 'object_tracking'
            self.nav_state_pub.publish(nav_state)
            self.timer = self.create_timer(1.0/self.motion_command_frequency, self.publish_motion_command)
        elif self.object_tracking == True and msg.label == self.not_found_value:
            #when object is lost -> go back to exploration
            self.object_tracking = False
            self.get_logger().info("Object lost. Going back to exploration state.")
            nav_state = String()
            nav_state.data = 'exploration'
            self.nav_state_pub.publish(nav_state)
            self.timer.destroy()
        elif self.object_tracking == True and msg.label == self.object_name:
            #continue tracking the object
            if ((msg.ymax - msg.ymin) / self.image_height) > 0.5:
                self.get_logger().info("Goal achieved")
                nav_state = String()
                nav_state.data = 'goal_achieved'
                self.nav_state_pub.publish(nav_state)
                self.motion_command = 'stop'
            else:
                nav_state = String()
                nav_state.data = 'object_tracking'
                self.nav_state_pub.publish(nav_state)
                if (msg.xmin + (msg.xmax - msg.xmin)/2.0) < ((self.image_width/2.0)):
                    self.motion_command = 'turn_left'
                elif (msg.xmin + (msg.xmax - msg.xmin)/2.0) > ((self.image_width/2.0)):
                    self.motion_command = 'turn_right'
                else:
                    self.motion_command = 'forward'
    def publish_motion_command(self):
        if self.nav_state != 'collision' and self.object_tracking == True:
            msg = String()
            msg.data = self.motion_command 
            self.motion_controller_pub.publish(msg)

    def nav_state_callback(self, msg):
        self.nav_state = msg.data

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
