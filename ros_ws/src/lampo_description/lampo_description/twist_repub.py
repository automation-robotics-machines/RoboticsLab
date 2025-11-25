#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Twist

class TwistConverter(Node):
    def __init__(self):
        super().__init__('twist_stamped_to_twist')

        # Declare and get the robot namespace parameter
        self.declare_parameter('robot_namespace', 'r1_')
        robot_ns = self.get_parameter('robot_namespace').get_parameter_value().string_value

        # Build topic names with namespace
        input_topic = f"/{robot_ns}/cmd_vel_smoothed"
        output_topic = f"/{robot_ns}/cmd_vel"

        # Subscriber and Publisher
        self.subscription = self.create_subscription(
            TwistStamped,
            input_topic,
            self.listener_callback,
            10
        )
        self.publisher = self.create_publisher(Twist, output_topic, 10)

        self.get_logger().info(f"Started topic repub for namespace: {robot_ns}")

    def listener_callback(self, msg: TwistStamped):
        twist_msg = Twist()
        twist_msg.linear = msg.twist.linear
        twist_msg.angular = msg.twist.angular
        self.publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TwistConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
