import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped

import math

class PointPublisher(Node):

    def __init__(self):
        super().__init__('map_joy_to_ack')

        self.subscription = self.create_subscription(Joy, 'joy', self.timer_callback, 10)
        self.subscription

        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'vehicle_command_ackermann', 10)

        self.max_speed = 0.5
        self.max_angle = math.radians(45)

    def timer_callback(self, data):
        msg = AckermannDriveStamped()

        #msg.header.stamp = self.get_clock().now().to_msg()

        speed_axis = data.axes[1]
        steering_axis = data.axes[3]
    
        msg.drive.speed = self.max_speed * speed_axis
        msg.drive.steering_angle = self.max_angle * steering_axis
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: speed={msg.drive.speed}, direction = {msg.drive.steering_angle}')


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