import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class PointPublisher(Node):

    def __init__(self):
        super().__init__('wall_follow_node')

        self.subscription = self.create_subscription(Joy, 'joy', self.timer_callback, 10)
        self.subscription

        self.subscription = self.create_subscription(LaserScan, 'scan', self.listener_callback, 10)
        self.subscription  

        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'vehicle_command_ackermann', 10)


    def timer_callback(self, data):
        msg = AckermannDriveStamped()


def main(args=None):
    rclpy.init(args=args)

    point_publisher = PointPublisher()

    rclpy.spin(point_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    point_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()