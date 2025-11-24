#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Twist

class TwistConverter(Node):
    def __init__(self):
        super().__init__('twist_stamped_to_twist')
        

        # Subscriber and Publisher
        self.subscription = self.create_subscription(
            TwistStamped,
            "/sweepee_1/cmd_vel_smoothed",
            self.listener_callback,
            10
        )
        self.publisher = self.create_publisher(Twist, "/sweepee_1/cmd_vel", 10)

        self.get_logger().info("Started topic repub")

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
