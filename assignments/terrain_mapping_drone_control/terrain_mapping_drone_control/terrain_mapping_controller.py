#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import math
import time
import numpy as np

from px4_msgs.msg import VehicleOdometry, OffboardControlMode, VehicleCommand, VehicleStatus, TrajectorySetpoint
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped

class TerrainMappingController(Node):
    """
    A ROS 2 node for controlling PX4 drone for terrain mapping using ORBSLAM3.
    This controller implements a lawnmower pattern trajectory over the Bishop Fault Scarp
    while maintaining a constant height and capturing images for SLAM.
    """
    def __init__(self):
        super().__init__('terrain_mapping_controller')

        # Configure QoS profile for PX4 communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Subscribers
        self.vehicle_odometry_subscriber = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', 
            self.vehicle_odometry_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self.vehicle_status_callback, qos_profile)
        self.orbslam_pose_subscriber = self.create_subscription(
            PoseStamped, '/orbslam3/camera_pose',
            self.orbslam_pose_callback, 10)

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_odometry = VehicleOdometry()
        self.vehicle_status = VehicleStatus()
        self.start_time = time.time()
        self.cv_bridge = CvBridge()
        self.current_slam_pose = None
        
        # Flight parameters
        self.MAPPING_HEIGHT = 20.0  # meters
        self.SURVEY_SPEED = 5.0  # m/s
        self.AREA_LENGTH = 100.0  # meters (X direction)
        self.AREA_WIDTH = 50.0  # meters (Y direction)
        self.STRIP_SPACING = 10.0  # meters between parallel survey lines
        self.HEIGHT_REACHED_THRESHOLD = 0.3  # meters
        
        # Survey pattern parameters
        self.num_strips = int(self.AREA_WIDTH / self.STRIP_SPACING) + 1
        self.current_strip = 0
        self.strip_direction = 1  # 1 for forward, -1 for backward
        
        # State machine
        self.state = "TAKEOFF"  # States: TAKEOFF, SURVEY, RTL
        
        # Control parameters
        self.height_P_gain = 2.0
        self.max_vertical_velocity = 2.0
        self.takeoff_velocity = 1.5

        # Create a timer to publish control commands
        self.create_timer(0.1, self.timer_callback)  # 10Hz control loop

    def vehicle_odometry_callback(self, msg):
        """Callback function for vehicle odometry data."""
        self.vehicle_odometry = msg

    def vehicle_status_callback(self, msg):
        """Callback function for vehicle status data."""
        self.vehicle_status = msg

    def orbslam_pose_callback(self, msg):
        """Callback function for ORBSLAM3 pose estimates."""
        self.current_slam_pose = msg

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info("Arm command sent")

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info("Disarm command sent")

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Offboard mode command sent")

    def publish_offboard_control_mode(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_trajectory_setpoint(self, x=0.0, y=0.0, z=0.0, yaw=0.0):
        """Publish trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        """Publish vehicle command."""
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def is_at_target_height(self):
        """Check if the drone has reached the target height."""
        current_height = -self.vehicle_odometry.position[2]
        return abs(current_height - self.MAPPING_HEIGHT) < self.HEIGHT_REACHED_THRESHOLD

    def calculate_survey_position(self):
        """Calculate the next position in the lawnmower pattern."""
        # Calculate Y position based on current strip
        y_pos = (self.current_strip * self.STRIP_SPACING) - (self.AREA_WIDTH / 2)
        
        # Calculate X position based on direction
        if self.strip_direction == 1:
            x_pos = self.AREA_LENGTH / 2
        else:
            x_pos = -self.AREA_LENGTH / 2
            
        return x_pos, y_pos

    def timer_callback(self):
        """Timer callback for control loop."""
        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()
            self.start_time = time.time()

        self.publish_offboard_control_mode()

        if self.state == "TAKEOFF":
            if not self.is_at_target_height():
                self.publish_trajectory_setpoint(
                    x=0.0,
                    y=0.0,
                    z=-self.MAPPING_HEIGHT,
                    yaw=0.0
                )
                self.get_logger().info(f"Taking off... Current height: {-self.vehicle_odometry.position[2]:.2f}m")
            else:
                self.state = "SURVEY"
                self.get_logger().info("Starting survey pattern")

        elif self.state == "SURVEY":
            x_target, y_target = self.calculate_survey_position()
            
            # Check if we've reached the end of the current strip
            current_x = self.vehicle_odometry.position[0]
            if (self.strip_direction == 1 and current_x >= self.AREA_LENGTH/2) or \
               (self.strip_direction == -1 and current_x <= -self.AREA_LENGTH/2):
                self.current_strip += 1
                self.strip_direction *= -1
                
                # Check if we've completed the survey
                if self.current_strip >= self.num_strips:
                    self.state = "RTL"
                    self.get_logger().info("Survey complete, returning to launch")
                    return

            self.publish_trajectory_setpoint(
                x=x_target,
                y=y_target,
                z=-self.MAPPING_HEIGHT,
                yaw=0.0  # Keep camera pointing forward
            )

            # Log progress
            progress = (self.current_strip * 100.0) / self.num_strips
            self.get_logger().info(
                f"Survey progress: {progress:.1f}%, "
                f"Strip: {self.current_strip + 1}/{self.num_strips}, "
                f"Height: {-self.vehicle_odometry.position[2]:.2f}m"
            )

        elif self.state == "RTL":
            self.publish_trajectory_setpoint(
                x=0.0,
                y=0.0,
                z=-self.MAPPING_HEIGHT,
                yaw=0.0
            )

        self.offboard_setpoint_counter += 1

def main(args=None):
    rclpy.init(args=args)
    controller = TerrainMappingController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Stopping controller...')
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 