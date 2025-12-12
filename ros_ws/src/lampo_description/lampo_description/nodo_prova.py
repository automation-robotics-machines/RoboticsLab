import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import threading
import math
import time
import numpy 



class NodoMio(Node):
    def __init__(self):
        super().__init__('minimal_publisher')

        self.publisher_   = self.create_publisher(Twist, '/r1_/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Odometry,
            '/r1_/odom',
            self.listener_callback,
            10)
        
        # Not needed because we publish cmd vel in the separate thread
        # timer_period = 0.5 
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.odom = None
        self.t1 = threading.Thread(target=self.algorithm)
        self.t1.start()


    def listener_callback(self, msg : Odometry):
        self.odom = msg

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)

    def algorithm(self):
        self.get_logger().info('Thread started')
        while(self.odom==None):
            time.sleep(0.1)
        self.get_logger().info('first odom msg received')
        while(math.fabs(self.odom.pose.pose.position.x) > 0.05):
            msg = Twist()
            msg.linear.x = 0.1 * -1 * numpy.sign(self.odom.pose.pose.position.x)
            self.publisher_.publish(msg)
            time.sleep(0.1)
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info('Thread finished')
        

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