import rclpy
import math
from rclpy.node import Node

from sensor_msgs.msg import LaserScan

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('lidar_reader')
        self.subscription = self.create_subscription(
            LaserScan, 'scan', self.listener_callback, 10)
        self.subscription  

        #self.publisher_ = self.create_publisher(LaserScan, 'lidar_display', 10)

    def listener_callback(self, msg):
        #self.get_logger().info(f'x = {msg.x}, y = {msg.y}, z = {msg.z}')
        #pub_msg = LaserScan()

        R2D = 180/(math.pi)

        min_angle = msg.angle_min * R2D
        angle_inc = msg.angle_increment * R2D

        beam_angle = -90
        angle_delta = 35

        i = round((beam_angle - min_angle) / angle_inc)

        numerical_inc = round(angle_delta / angle_inc)

        d1 = msg.ranges[i]
        d2 = msg.ranges[i+numerical_inc]

        #self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: d1 = {d1} m, d2 = {d2} m, i = {i}, num_inc = {numerical_inc} ')


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()