#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import control
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from rcl_interfaces.msg import Parameter, ParameterDescriptor, FloatingPointRange
from rclpy.parameter import ParameterType

class CartPoleLQR(Node):
    def __init__(self):
        super().__init__('cart_pole_lqr')
        
        # System parameters
        self.declare_parameter('mass_cart', 1.0)
        self.declare_parameter('mass_pole', 0.1)
        self.declare_parameter('pole_length', 1.0)
        self.declare_parameter('gravity', 9.81)
        
        # LQR weights
        self.declare_parameter('Q_x', 1.0)
        self.declare_parameter('Q_x_dot', 1.0)
        self.declare_parameter('Q_theta', 1.0)
        self.declare_parameter('Q_theta_dot', 1.0)
        self.declare_parameter('R', 1.0)
        
        # Control limits
        self.declare_parameter('force_limit', 100.0)

        # Get initial parameters
        self.update_parameters()
        
        # System state: [x, x_dot, theta, theta_dot]
        self.state = np.array([0.0, 0.0, 0.1, 0.0])  # Small initial angle for testing
        
        # Setup publishers for control commands
        self.cart_cmd_pub = self.create_publisher(Float64, '/cart_pole/cart_slider_cmd', 10)
        
        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/cart_pole/joint_states',
            self.joint_state_callback,
            10)
        
        # Create timer for control loop
        self.create_timer(0.02, self.control_loop)  # 50Hz control loop
        
        # Compute initial LQR gains
        self.compute_lqr_gains()
        
        # Add parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)

    def update_parameters(self):
        # System parameters
        self.M = self.get_parameter('mass_cart').value
        self.m = self.get_parameter('mass_pole').value
        self.L = self.get_parameter('pole_length').value
        self.g = self.get_parameter('gravity').value
        
        # Control limit
        self.u_max = self.get_parameter('force_limit').value
        
        # Update Q matrix with all configurable weights
        self.Q = np.diag([
            self.get_parameter('Q_x').value,
            self.get_parameter('Q_x_dot').value,
            self.get_parameter('Q_theta').value,
            self.get_parameter('Q_theta_dot').value
        ])
        self.R = np.array([[self.get_parameter('R').value]])

    def compute_lqr_gains(self):
        try:
            # System parameters
            M = self.M  # Cart mass
            m = self.m  # Pendulum mass
            L = self.L  # Pendulum length
            g = self.g  # Gravity
            
            # Derived parameters
            I = m * L * L / 3  # Moment of inertia of the pendulum
            p = I * (M + m) + M * m * L * L  # Denominator term
            
            # Linearized dynamics matrix A for inverted pendulum
            A = np.array([
                [0, 1, 0, 0],
                [0, 0, -m * m * g * L * L / p, 0],
                [0, 0, 0, 1],
                [0, 0, -m * g * L * (M + m) / p, 0]
            ])
            
            # Input matrix B
            B = np.array([
                [0],
                [(I + m * L * L) / p],
                [0],
                [m * L / p]
            ])
            
            # Compute LQR gains
            K, S, E = control.lqr(A, B, self.Q, self.R)
            self.K = K
            
            # Log the gains for debugging
            self.get_logger().info(f'LQR gains computed: K = [{K[0,0]:.2f}, {K[0,1]:.2f}, {K[0,2]:.2f}, {K[0,3]:.2f}]')
            
        except Exception as e:
            self.get_logger().error(f'Failed to compute LQR gains: {e}')

    def joint_state_callback(self, msg):
        try:
            # Update state from joint positions and velocities
            cart_idx = msg.name.index('cart_slider')
            pole_idx = msg.name.index('pole_hinge')
            
            self.state[0] = msg.position[cart_idx]  # cart position
            self.state[1] = msg.velocity[cart_idx]  # cart velocity
            theta = msg.position[pole_idx]  # pole angle
            
            # Better angle normalization using arctan2
            self.state[2] = np.arctan2(np.sin(theta), np.cos(theta))
            self.state[3] = msg.velocity[pole_idx]  # pole angular velocity
            
            # Log state for debugging
            self.get_logger().debug(
                f'State - x: {self.state[0]:.2f}m, '
                f'x_dot: {self.state[1]:.2f}m/s, '
                f'theta: {self.state[2]:.2f}rad, '
                f'theta_dot: {self.state[3]:.2f}rad/s'
            )
            
        except ValueError as e:
            self.get_logger().warn(f'Joint state callback error: {e}')

    def control_loop(self):
        try:
            # Compute LQR control input
            u = -self.K @ self.state
            
            # Apply saturation limits
            u_saturated = np.clip(u[0], -self.u_max, self.u_max)
            
            # Log control and state information
            self.get_logger().info(
                f'Position: {self.state[0]:.2f}m, '
                f'Force: {u_saturated:.2f}N, '
                f'Theta: {self.state[2]:.2f}rad'
            )
            
            # Publish force command
            cmd_msg = Float64()
            cmd_msg.data = float(u_saturated)
            self.cart_cmd_pub.publish(cmd_msg)
            
        except Exception as e:
            self.get_logger().error(f'Control loop error: {e}')

    def parameter_callback(self, params):
        try:
            for param in params:
                if param.name in ['mass_cart', 'mass_pole', 'pole_length', 'gravity',
                                'Q_x', 'Q_x_dot', 'Q_theta', 'Q_theta_dot', 'R',
                                'force_limit']:
                    self.update_parameters()
                    self.compute_lqr_gains()
            return True
        except Exception as e:
            self.get_logger().error(f'Parameter update failed: {e}')
            return False

def main(args=None):
    rclpy.init(args=args)
    node = CartPoleLQR()
    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f'Node error: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 