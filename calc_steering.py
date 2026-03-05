import rclpy
from rclpy.node import Node

import math
import numpy as np
import time

from geometry_msgs.msg import Point, Twist, PoseStamped, TransformStamped
from ackermann_msgs.msg import AckermannDriveStamped

from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

D2R = math.pi/180.0
R2D = 180.0/math.pi

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('calc_steering')
        self.subscription = self.create_subscription(
            PoseStamped, 'vehicle_pose', self.pose_came_in_callback, 10)
        self.subscription  # prevent unused variable warning

        self.ackerman_publisher = self.create_publisher(AckermannDriveStamped, 'vehicle_command_ackermann', 10)

        self.declare_parameter('theta_deg', 42.0)
        self.declare_parameter('d_LookAhead', 2.5)
        self.declare_parameter('d_desired', 1.0)

        self.steering_ang_rad = 0.0
        self.declare_parameter('speed', 0.0)

        self.declare_parameter('starting_delay', 0.0)

        self.declare_parameter('save_to_file', False)
        self.declare_parameter('file_name', 'my_default.csv')

        self.max_speed = 6.0
        self.max_ang_deg = 45.0 

        self.declare_parameter('Kp', 0.5)
        self.declare_parameter('Ki', 0.1)
        self.declare_parameter('Kd', 0.1)
        self.error = 0.0
        self.error_m1 = 0.0
        self.error_m2 = 0.0

        # publish a Marker Array for the wall for rviz
        self.publisher_ = self.create_publisher(Marker, 'visualization_marker', 10)
        self.timer = self.create_timer(timer_period_sec=1.0, callback=self.timer_callback)

        self.id_counter = 0

        self.start_time = time.time()

        self.print_stuff = self.get_parameter('save_to_file').value
        self.filename = self.get_parameter('file_name').value

        if self.print_stuff:
            self.fp = open(self.filename, "w")
            self.fp.write('time (sec), d_wall (m), heading_error_deg, steering_ang_deg\n')

    def timer_callback(self):
        #self.get_logger().info(f'\nREADY TO DRAW WALL\n')

        d_desired = self.get_parameter('d_desired').value
        marker0 = 0
        self.publish_line(0.0, 0.0, 50.0, 0.0, 0.0, 1.0, 1.0, marker0)
        self.publish_line(0.0, d_desired, 50.0, d_desired, 0.0, 1.0, 0.0, marker0)

    def pose_came_in_callback(self, msg):
        
        theta_rad = self.get_parameter('theta_deg').value * D2R
        d_LookAhead = self.get_parameter('d_LookAhead').value
        d_desired = self.get_parameter('d_desired').value

        my_quat = msg.pose.orientation
        my_pos  = msg.pose.position
        
        P_sensor_origin = np.array([my_pos.x, my_pos.y, 0.0])
        beta_rad = 2.0*math.atan2(my_quat.z, my_quat.w)
        
        d1, d2, P1, P2 = get_d1_d2(P_sensor_origin, beta_rad, theta_rad)

        d_wall, delta_e = get_steering_error(d1, d2, theta_rad, d_LookAhead, d_desired)

        self.error_m2 = self.error_m1
        self.error_m1 = self.error
        self.error = delta_e

        # draw the laser lines
        self.publish_line(my_pos.x, my_pos.y, P1[0], P1[1], 1.0, 1.0, 0.0, 2001)
        self.publish_line(my_pos.x, my_pos.y, P2[0], P2[1], 1.0, 1.0, 0.0, 2002)

        #self.get_logger().info(f'd1 = {d1} m, d2 = {d2}, d_wall = {d_wall}, delta_e = {delta_e*R2D} deg')

        # CONTROLLER CODE HERE
        # update steering angle and publish the Ackerman message


        # speed parameter will be initialized to zero.  It will
        # be changed from the command line.
        speed = self.get_parameter('speed').value
        
        if speed > self.max_speed:
            speed = self.max_speed

        wait_time =self.get_parameter('starting_delay').value
        if time.time() - self.start_time < wait_time:
            speed = 0.0

        Kp = self.get_parameter('Kp').value
        Ki = self.get_parameter('Ki').value
        Kd = self.get_parameter('Kd').value

        delta_ang_rad = Kp * (self.error - self.error_m1) + Ki * self.error + Kd * (self.error - 2*self.error_m1 + self.error_m2)

        if delta_ang_rad > (45 * D2R):
            delta_ang_rad = 45 * D2R
        elif delta_ang_rad < (-45 * D2R):
            delta_ang_rad = -45 * D2R

        ang_rad = 0.0
        ang_rad += delta_ang_rad
        self.steering_ang_rad = ang_rad
        # publish the Ackerman message
        out_msg = AckermannDriveStamped()
        out_msg.drive.steering_angle = ang_rad
        out_msg.drive.speed = speed
        self.ackerman_publisher.publish(out_msg)

        now = time.time() - self.start_time
        if self.print_stuff and now>=wait_time:
            #time (sec), d_wall (m), heading_error_deg, steering_ang_deg
            self.fp.write(f'{now}, {d_wall}, {delta_e*R2D}, {self.steering_ang_rad*R2D}\n')


    def publish_line(self, x1, y1, x2, y2, r, g, b, marker_id):
        marker = Marker()
        marker.header.frame_id = "utm" # Ensure this frame exists in your TF tree
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "line_namespace"
        if marker_id == 0:
            marker.id = self.id_counter
            self.id_counter += 1
        else:
            marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        # Set the scale (line width)
        marker.scale.x = 0.1 # meters

        # Set the color (r, g, b, a) - alpha (a) must be non-zero
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 1.0 # Must set alpha for visibility

        # Define the points of the line
        p1 = Point()
        p1.x = x1
        p1.y = y1
        p1.z = 0.0
        p2 = Point()
        p2.x = x2
        p2.y = y2
        p2.z = 0.0

        marker.points.append(p1)
        marker.points.append(p2)
        
        # Publish the marker
        self.publisher_.publish(marker)

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

################################################
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
################################################
def intersection_of_two_lines(S1, SOL1, S2, SOL2):
    
    term1 = np.cross(S2, SOL2)
    term2 = -np.cross(np.dot(S1,S2)*S1, SOL2)
    term3 = np.dot(np.cross(S1, SOL1), S2)*S2
    numer = term1 + term2 + term3
    
    denom = 1 - (np.dot(S1, S2))**2
    
    return(numer/denom)
# --------------------------------
def get_d1_d2(P_sensor_origin, beta_rad, theta_rad):
    # Will calculate the distances d1 and d2 from the 
    # origin of the sensor coord sys to a wall along
    # the x axis.  The ranging beams are separated by 
    # the angle theta.
    #
    # NOTE:  P_sensor_origin is a length 3 numpy array with z=0.
    
    debug_print = False
    
    S_wall = np.array([1.0,0.0,0.0])
    SOL_wall = np.array([0.0,0.0,0.0])
    
    beam1_ang = beta_rad + 1.5*math.pi
    S_beam1 = np.array([math.cos(beam1_ang), math.sin(beam1_ang), 0.0])
    SOL_beam1 = np.cross(P_sensor_origin, S_beam1)
    
    beam2_ang = beam1_ang + theta_rad
    S_beam2 = np.array([math.cos(beam2_ang), math.sin(beam2_ang), 0.0])
    SOL_beam2 = np.cross(P_sensor_origin, S_beam2)
    
    P1 = intersection_of_two_lines(S_beam1, SOL_beam1, S_wall, SOL_wall)
    P2 = intersection_of_two_lines(S_beam2, SOL_beam2, S_wall, SOL_wall)
    
    if debug_print:
        print(f'\nP1 = {P1}\nP2 = {P2}\n')
    
    d1 = np.linalg.norm(P1-P_sensor_origin)
    d2 = np.linalg.norm(P2-P_sensor_origin)
    
    return (d1, d2, P1, P2)
################################################
