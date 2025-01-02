import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.odom_publisher_ = self.create_publisher(Odometry, 'odometry/filtered', 10)
        self.laser_publisher_ = self.create_publisher(LaserScan, 'scan', 10)

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        odom_msg = Odometry()
        #odom_msg.data = 'Hello World: %d' % self.i
        self.odom_publisher_.publish(odom_msg)
        
        laser_msg = LaserScan()
        #odom_msg.data = 'Hello World: %d' % self.i
        self.laser_publisher_.publish(laser_msg)

        self.get_logger().info('Publishing: "%i"' % self.i)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()