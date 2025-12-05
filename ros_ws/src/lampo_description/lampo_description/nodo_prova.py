import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import String
from nav_msgs.msg import Odometry




class NodoMio(Node):
    def __init__(self):
        super().__init__('minimal_publisher')

        # self.publisher_   = self.create_publisher(String, 'topic', 10)
        self.subscription = self.create_subscription(
            Odometry,
            '/r1_/odom',
            self.listener_callback,
            10)
        
        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.i = 0

    def listener_callback(self, msg : Odometry):
        self.get_logger().info('x: "%f"' % msg.pose.pose.position.x)
        self.get_logger().info('y: "%f"' % msg.pose.pose.position.y)
        self.get_logger().info('z: "%f"' % msg.pose.pose.position.z)
        self.get_logger().info('\n\n')

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    try:
        with rclpy.init(args=args):
            minimal_publisher = NodoMio()
            rclpy.spin(minimal_publisher)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    finally:
        pass


if __name__ == '__main__':
    main()