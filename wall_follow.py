import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

import math
import numpy as np

class PointPublisher(Node):

    def __init__(self):
        super().__init__('wall_follow_node')

        self.subscription = self.create_subscription(Joy, 'joy', self.timer_callback, 10)
        self.subscription

        self.subscription = self.create_subscription(LaserScan, 'scan', self.listener_callback, 10)
        self.subscription  

        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'vehicle_command_ackermann', 10)

    def get_steering_error(d1, d2, theta_rad, d_LookAhead, d_desired):
        # d1, d2, d_LookAhead, and d_desired have units of m
        # theta_rad is in units of radians (dimensionless)
        
        # returns the distance to wall and the orientation error to
        # point towards the look-ahead point
        
        debug_print = False
        
        # get coordinates of points P1 and P2 in sensor coord sys
        P1 = np.array([0, -d1])
        x2 = d2*math.cos(1.5*math.pi + theta_rad) # added 270 deg to theta
        y2 = d2*math.sin(1.5*math.pi + theta_rad)
        P2 = np.array([x2, y2])
        if debug_print:
            print(f'\nP1 = {P1}\nP2 = {P2}\n')
        
        v_vec = (P2-P1)/np.linalg.norm(P2-P1)
        x_sensor = np.array([1.0, 0.0])
        y_sensor = np.array([0.0, 1.0])
        cos_phi = np.dot(x_sensor, v_vec)
        d_wall = d1 * cos_phi
        if debug_print:
            print(f'v_vec = {v_vec}\n')
            print(f'd_wall = {d_wall}\n')
        
        S_perp = np.array([-v_vec[1], v_vec[0]])
        P_LookAhead = d_LookAhead * v_vec + (d_desired - d_wall)*S_perp
        if debug_print:
            print(f'P_LookAhead = {P_LookAhead}')
        
        S_LookAhead = P_LookAhead/np.linalg.norm(P_LookAhead)
        
        delta_e = math.atan2(S_LookAhead[1], S_LookAhead[0])
        # atan2 function will bound delta_e between -pi and +pi
        return (d_wall, delta_e)

    def timer_callback(self, data, msg):
        msg = AckermannDriveStamped()

        speed = 2.0

        #Monitor buttons on joystick for start/stop
        start_button = data.button[0]
        stop_button = data.button[1]

        if start_button == 1:
            msg.drive.speed = speed
        elif stop_button == 1:
            msg.drive.speed = 0

        #Calculate current distance from wall and delta_e
        R2D = 180/(math.pi)

        min_angle = msg.angle_min * R2D
        angle_inc = msg.angle_increment * R2D
        delta_t = msg.time_increment

        beam_angle = -90
        theta = 35
        theta_rad = math.radians(theta)

        i = round((beam_angle - min_angle) / angle_inc)

        numerical_inc = round(theta / angle_inc)

        d1 = msg.ranges[i]
        d2 = msg.ranges[i+numerical_inc]

        d_LookAhead = 2.5
        d_desired = 1

        [d_wall,delta_e] = self.get_steering_error(d1, d2, theta_rad, d_LookAhead, d_desired) #Gets dwall and delta_e

        #PID Control
        k_p = 1.0
        k_i = 0.0
        k_d = 0.0

        u_k = k_p * delta_e + k_i * delta_t * delta_e + (k_d / delta_t) * (delta_e)

        if u_k > math.radians(45):
            u_k = math.radians(45)
        elif u_k < math.radians(-45):
            u_k = math.radians(-45)

        msg.drive.steering_angle = u_k

        #Publish ackermann msg with new steering angle
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: dwall = {d_wall}, delta_e = {delta_e}, u_k = {u_k}')


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
