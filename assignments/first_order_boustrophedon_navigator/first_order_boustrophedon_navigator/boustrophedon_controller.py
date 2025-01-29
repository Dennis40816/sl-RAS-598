import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import numpy as np
import math
from collections import deque
from std_msgs.msg import Float64
import json


class RosNodeDestroy(Exception):
    """Custom exception to trigger ROS node destruction."""
    pass


class Performance:
    def __init__(self):
        self.cross_track_errors = deque(maxlen=1000)
        self.path_length = 0.0
        self.covered_points = set()
        self.target_coverage_points = 0
        self.start_time = None
        self.end_time = None

    def update_cross_track_error(self, error):
        self.cross_track_errors.append(abs(error))

    def update_path_length(self, distance):
        self.path_length += distance

    def add_covered_point(self, point):
        self.covered_points.add(point)

    def start_timer(self, current_time):
        if self.start_time is None:
            self.start_time = current_time

    def end_timer(self, current_time):
        self.end_time = current_time

    def summarize(self):
        avg_error = sum(self.cross_track_errors) / \
            len(self.cross_track_errors) if self.cross_track_errors else 0.0
        max_error = max(
            self.cross_track_errors) if self.cross_track_errors else 0.0
        min_error = min(
            self.cross_track_errors) if self.cross_track_errors else 0.0
        coverage = len(self.covered_points)
        time_taken = (self.end_time - self.start_time).nanoseconds / \
            1e9 if self.start_time and self.end_time else 0.0

        return {
            'average_error': avg_error,
            'max_error': max_error,
            'min_error': min_error,
            'path_length': self.path_length,
            'coverage_points': coverage,
            'time_taken': time_taken,
            'target_coverage_points': self.target_coverage_points
        }


class BoustrophedonController(Node):
    def __init__(self):
        super().__init__('lawnmower_controller')

        # Declare parameters with dict type
        # WARN: should sync with boustrophedon.launch.py
        self.default_gains = {
            'Kp_linear': 4,
            'Kd_linear': 0.3,
            'Kp_angular': 6.5,
            'Kd_angular': 0.05
        }
        
        # self.default_gains = {
        #     'Kp_linear': 10.0,
        #     'Kd_linear': 0.1,
        #     'Kp_angular': 5.0,
        #     'Kd_angular': 0.2
        # }

        str_default_gains = json.dumps(self.default_gains)

        self.declare_parameter('controller_gains', str_default_gains)

        # Get parameters
        # usage example:
        # ros2 launch first_order_boustrophedon_navigator boustrophedon.launch.py controller_gains:='{"Kp_linear": 15.0, "Kd_linear": 0.2}'
        str_controller_gains = self.get_parameter('controller_gains').value

        try:
            self.get_logger().warn(str_controller_gains)
            gains = json.loads(str_controller_gains)
            
        except json.JSONDecodeError:
            self.get_logger().warn("Invalid JSON for 'controller_gains'. Using default gains.")
            gains = json.loads(self.default_gains)

        self.Kp_linear = gains.get(
            'Kp_linear', self.default_gains['Kp_linear'])
        self.Kd_linear = gains.get(
            'Kd_linear', self.default_gains['Kd_linear'])
        self.Kp_angular = gains.get(
            'Kp_angular', self.default_gains['Kp_angular'])
        self.Kd_angular = gains.get(
            'Kd_angular', self.default_gains['Kd_angular'])

        # Log gain
        self.get_logger().warn(f"Kp_linear: {self.Kp_linear}")
        self.get_logger().warn(f"Kd_linear: {self.Kd_linear}")
        self.get_logger().warn(f"Kp_angular: {self.Kp_angular}")
        self.get_logger().warn(f"Kd_angular: {self.Kd_angular}")

        # Create publisher and subscriber
        self.velocity_publisher = self.create_publisher(
            Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, 10)
        self.error_publisher = self.create_publisher(
            Float64, '/cross_track_error', 10)

        # Initialize state variables
        self.pose = Pose()
        self.prev_linear_error = 0.0
        self.prev_angular_error = 0.0
        self.prev_time = self.get_clock().now()

        # Waypoints
        self.spacing = 1.0
        self.waypoints = self.generate_waypoints()
        self.current_waypoint = 0

        # Performance tracker
        self.performance = Performance()
        self.performance.target_coverage_points = len(self.waypoints)

        # Create control loop timer
        self.timer = self.create_timer(0.1, self.control_loop)
        self.performance.start_timer(self.get_clock().now())

        self.get_logger().info('Lawnmower controller started')

    def generate_waypoints(self):
        waypoints = []
        y = 8.0

        while y >= 3.0:
            if len(waypoints) % 2 == 0:
                waypoints.append((2.0, y))
                waypoints.append((9.0, y))
            else:
                waypoints.append((9.0, y))
                waypoints.append((2.0, y))

            y -= self.spacing

        return waypoints

    def calculate_cross_track_error(self):
        if self.current_waypoint < 1:
            return 0.0

        start = np.array(self.waypoints[self.current_waypoint - 1])
        end = np.array(self.waypoints[self.current_waypoint])
        pos = np.array([self.pose.x, self.pose.y])

        path_vector = end - start
        path_length = np.linalg.norm(path_vector)
        if path_length < 1e-6:
            return np.linalg.norm(pos - start)

        path_unit = path_vector / path_length
        pos_vector = pos - start
        projection_length = np.dot(pos_vector, path_unit)
        projection_length = max(0, min(path_length, projection_length))
        projected_point = start + projection_length * path_unit

        error_vector = pos - projected_point
        error_sign = np.sign(
            np.cross(path_unit, error_vector / np.linalg.norm(error_vector)))
        error = np.linalg.norm(error_vector) * error_sign

        self.performance.update_cross_track_error(error)

        # Publish cross-track error
        # TODO: Is this should be abs(error)?
        error_msg = Float64()
        error_msg.data = error
        self.error_publisher.publish(error_msg)

        return error

    def pose_callback(self, msg):
        self.pose = msg

    def get_distance(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def get_angle(self, x1, y1, x2, y2):
        return math.atan2(y2 - y1, x2 - x1)

    def control_loop(self):

        # navigation process completed
        if self.current_waypoint >= len(self.waypoints):
            self.get_logger().info('Lawnmower pattern complete')

            self.performance.end_timer(self.get_clock().now())
            summary = self.performance.summarize()

            self.get_logger().info(f"Performance Summary: {summary}")

            self.timer.cancel()
            raise RosNodeDestroy('Navigation Ended')

        cross_track_error = self.calculate_cross_track_error()

        target_x, target_y = self.waypoints[self.current_waypoint]
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds / 1e9

        distance = self.get_distance(
            self.pose.x, self.pose.y, target_x, target_y)
        target_angle = self.get_angle(
            self.pose.x, self.pose.y, target_x, target_y)
        angular_error = target_angle - self.pose.theta

        while angular_error > math.pi:
            angular_error -= 2 * math.pi
        while angular_error < -math.pi:
            angular_error += 2 * math.pi

        linear_error_derivative = (distance - self.prev_linear_error) / dt
        angular_error_derivative = (
            angular_error - self.prev_angular_error) / dt

        linear_velocity = self.Kp_linear * distance + \
            self.Kd_linear * linear_error_derivative
        angular_velocity = self.Kp_angular * angular_error + \
            self.Kd_angular * angular_error_derivative

        vel_msg = Twist()
        vel_msg.linear.x = min(linear_velocity, 2.0)
        vel_msg.angular.z = angular_velocity
        self.velocity_publisher.publish(vel_msg)

        # FIXME: don't use linear_velocity
        self.performance.update_path_length(linear_velocity * dt)

        self.prev_linear_error = distance
        self.prev_angular_error = angular_error
        self.prev_time = current_time

        if distance < 0.1:
            self.current_waypoint += 1
            self.performance.add_covered_point(
                (round(self.pose.x, 1), round(self.pose.y, 1)))
            self.get_logger().info(f'Reached waypoint {self.current_waypoint}')


def main(args=None):
    rclpy.init(args=args)
    controller = BoustrophedonController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    except RosNodeDestroy:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
