import math
import numpy as np

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int8

from gps_nav_interfaces.msg import CurrentGoalPose

from gps_nav.controller_plotter import CrossTrackPlotLogger
from gps_nav.uf_support.route_support import get_cross_track_and_heading_error


def wrap_angle(angle):
    """Wrap angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class VehicleController(Node):
    def __init__(self):
        super().__init__("vehicle_controller")

        self.declare_parameter("L_wheelbase_m", 0.33)
        self.declare_parameter("stanley_k", 2.0)
        self.declare_parameter("softening_speed", 0.2)
        self.declare_parameter("max_steering_rad", 0.45)

        self.subscription1 = self.create_subscription(
            PoseStamped, "vehicle_pose", self.vehicle_pose_callback, 1
        )

        self.subscription2 = self.create_subscription(
            CurrentGoalPose, "current_goal_pose", self.current_goal_pose_callback, 1
        )

        self.subscription3 = self.create_subscription(
            Int8, "e_stop", self.e_stop_callback, 10
        )

        self.publisher = self.create_publisher(
            AckermannDriveStamped, "vehicle_command_ackermann", 10
        )

        # 10 Hz timer
        self.main_timer = self.create_timer(
            timer_period_sec=0.1, callback=self.main_timer_callback
        )

        # data from subscriptions
        self.current_goal_point = np.array([0.0, 0.0, 0.0])
        self.current_goal_heading_rad = 0.0
        self.closest_point = np.array([0.0, 0.0, 0.0])
        self.closest_heading_rad = 0.0
        self.speed = 0.0
        self.state = 0.0

        self.vehicle_point = np.array([0.0, 0.0, 0.0])
        self.vehicle_heading_rad = 0.0

        self.pause = False
        self.last_pause_value = False

        self.have_vehicle_pose = False
        self.have_goal_pose = False

        self.plot_logger = CrossTrackPlotLogger(self, "stanley")

    def vehicle_pose_callback(self, msg):
        self.have_vehicle_pose = True

        self.vehicle_point[0] = msg.pose.position.x
        self.vehicle_point[1] = msg.pose.position.y
        self.vehicle_point[2] = 0.0

        self.vehicle_heading_rad = 2.0 * math.atan2(
            msg.pose.orientation.z, msg.pose.orientation.w
        )

    def current_goal_pose_callback(self, msg):
        self.have_goal_pose = True

        self.current_goal_point[0] = msg.current_goal_pose.pose.position.x
        self.current_goal_point[1] = msg.current_goal_pose.pose.position.y
        self.current_goal_point[2] = 0.0

        self.current_goal_heading_rad = 2.0 * math.atan2(
            msg.current_goal_pose.pose.orientation.z,
            msg.current_goal_pose.pose.orientation.w
        )

        self.closest_point[0] = msg.closest_pose.pose.position.x
        self.closest_point[1] = msg.closest_pose.pose.position.y
        self.closest_point[2] = 0.0

        self.closest_heading_rad = 2.0 * math.atan2(
            msg.closest_pose.pose.orientation.z,
            msg.closest_pose.pose.orientation.w
        )

        self.speed = msg.speed
        self.state = msg.state

        if self.speed > 2.5:
            self.speed = 2.5
        elif self.speed < -2.5:
            self.speed = -2.5

    def e_stop_callback(self, msg):
        if msg.data == 0:
            self.pause = True
        elif msg.data == 1:
            self.pause = False

    def main_timer_callback(self):
        if self.pause:
            out_msg = AckermannDriveStamped()
            out_msg.drive.speed = 0.0
            out_msg.drive.steering_angle = 0.0
            self.publisher.publish(out_msg)
            return

        if self.have_goal_pose and self.have_vehicle_pose:  # Stanley Controller
            k = self.get_parameter("stanley_k").value
            softening_speed = self.get_parameter("softening_speed").value
            max_steering_rad = self.get_parameter("max_steering_rad").value

            cross_track_error, heading_error, line_pt = get_cross_track_and_heading_error(
                self.closest_point,
                self.closest_heading_rad,
                self.vehicle_point,
                self.vehicle_heading_rad
            )

            # Use magnitude of speed in denominator to avoid sign flip issues
            v = abs(self.speed)

            # Stanley law
            cross_track_term = math.atan2(k * cross_track_error, v + softening_speed)
            angle_rad = wrap_angle(heading_error + cross_track_term)

            # Clamp steering angle
            angle_rad = max(-max_steering_rad, min(max_steering_rad, angle_rad))

            out_msg = AckermannDriveStamped()
            out_msg.drive.speed = self.speed
            out_msg.drive.steering_angle = angle_rad

            self.publisher.publish(out_msg)

            self.plot_logger.log(
                cross_track_error,
                heading_error,
                angle_rad,
                self.speed,
            )

    def destroy_node(self):
        self.plot_logger.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    vehicle_controller = VehicleController()

    try:
        rclpy.spin(vehicle_controller)
    except KeyboardInterrupt:
        pass

    vehicle_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
