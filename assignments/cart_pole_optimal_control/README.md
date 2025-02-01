![image](https://github.com/user-attachments/assets/b77c1fbb-7a51-4c9f-bc19-c43c981701d6)


# Cart-Pole Optimal Control Assignment

This ROS 2 package implements an LQR (Linear Quadratic Regulator) controller for balancing an inverted pendulum on a cart. The system demonstrates optimal control of an underactuated system, where we control only the cart's position to stabilize both the cart position and pole angle.

## Note

- [Hackmd](https://hackmd.io/@dennis40816/HJcpJjddyl)

## System Description

The cart-pole system consists of:
- A cart that can move horizontally along a rail
- A pole attached to the cart with a revolute joint
- Control input: Force applied to the cart
- State vector: [x, ẋ, θ, θ̇] (cart position, cart velocity, pole angle, pole angular velocity)

## Prerequisites

We use ROS2 Humble instead of ROS2 Jazzy for using Ubuntu 22.04.

**Already Existed (assignment-1)**

- ~~ROS 2 Jazzy~~ ROS2 Humble
- ~~Gazebo Garden~~ Gazebo Fortress

**Installation Required**

- Required ROS 2 packages:
  ```bash
  sudo apt install \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-sim \
    ros-humble-ros-gz-interfaces
  ```

- Python Control Systems Library:

   ```bash
   ## Not found so abort
   # sudo apt-get install python3-control

   # we use virtualenvwrapper
   workon <venv>
   pip install slycot
   pip install control
   ```

## Installation

1. Clone this repository into your ROS 2 workspace:

   ```bash
   cd ~/ros2_ws/src
   ln -s <cart_pole_optimal_control_path> .
   ```

2. Build the package:

   ```bash
   cd ~/ros2_ws
   # do not use --symlink-install
   colcon build --packages-select cart_pole_optimal_control
   ```

3. Source the workspace:

   ```bash
   source install/setup.bash
   ```

## Usage

Launch the simulation with:
```bash
ros2 launch cart_pole_optimal_control cart_pole_gazebo.launch.py
```

### Tuning Controller Parameters

The LQR controller can be tuned using ROS 2 parameters:
```bash
# Adjust position control weight
ros2 param set /cart_pole_lqr Q_x 2.0

# Adjust angle control weight
ros2 param set /cart_pole_lqr Q_theta 20.0

# Adjust control effort penalty
ros2 param set /cart_pole_lqr R 0.5
```

### System Parameters
- `mass_cart`: Mass of the cart (default: 1.0 kg)
- `mass_pole`: Mass of the pole (default: 0.1 kg)
- `pole_length`: Length of the pole (default: 1.0 m)
- `gravity`: Gravitational acceleration (default: 9.81 m/s²)

### Control Parameters
- `Q_x`: Weight for cart position error
- `Q_x_dot`: Weight for cart velocity error
- `Q_theta`: Weight for pole angle error
- `Q_theta_dot`: Weight for pole angular velocity error
- `R`: Weight for control effort

## Testing the Controller

1. Monitor system state:
   ```bash
   ros2 topic echo /cart_pole/joint_states
   ```

2. Apply disturbances:
   ```bash
   ros2 topic pub --once /cart_pole/cart_slider_cmd std_msgs/msg/Float64 "data: 10.0"
   ```

## File Structure

- `cart_pole_lqr.py`: Main LQR controller implementation
- `model.sdf`: Gazebo model definition
- `cart_pole_gazebo.launch.py`: Launch file for Gazebo simulation
- `package.xml`: Package dependencies
- `setup.py`: Package setup and entry points

## Theory

The LQR controller minimizes the quadratic cost function:

$$ J = \int_{0}^{\infty} (x^T Q x + u^T R u) dt $$

where:
- $x = [x, \dot{x}, \theta, \dot{\theta}]^T$ is the state vector
- $u$ is the control input (force on cart)
- $Q \in \mathbb{R}^{4\times4}$ is the state cost matrix
- $R \in \mathbb{R}$ is the control cost scalar

The nonlinear equations of motion for the cart-pole system are:

$$ \ddot{x} = \frac{F + ml\sin(\theta)(\dot{\theta}^2 - \frac{g}{l}\cos(\theta))}{M + m\sin^2(\theta)} $$

$$ \ddot{\theta} = \frac{-F\cos(\theta) - ml\dot{\theta}^2\cos(\theta)\sin(\theta) + (M+m)g\sin(\theta)}{l(M + m\sin^2(\theta))} $$

where:
- $M$ is the cart mass
- $m$ is the pole mass
- $l$ is the pole length
- $g$ is the gravitational acceleration
- $F$ is the applied force

**If you're interested in the derivative process, please refer to [here](#appendix-a-inverted-pendulum-dynamic-derivation)**

The system is linearized around the unstable equilibrium point $\theta = 0$ (upright position). For small deviations from vertical, $\sin(\theta) \approx \theta$ and $\cos(\theta) \approx 1$, giving the linear state-space model:

$$ \dot{x} = Ax + Bu $$

where:

$$ A = \begin{bmatrix} 
0 & 1 & 0 & 0 \\
0 & 0 & \frac{mg}{M} & 0 \\
0 & 0 & 0 & 1 \\
0 & 0 & \frac{(M+m)g}{Ml} & 0
\end{bmatrix} $$

$$ B = \begin{bmatrix}
0 \\
\frac{1}{M} \\
0 \\
-\frac{1}{Ml}
\end{bmatrix} $$

The LQR controller computes optimal feedback gains $K$ such that $u = -Kx$ minimizes the cost function. The resulting closed-loop system is:

$$ \dot{x} = (A - BK)x $$

The gain matrix $K$ is computed by solving the algebraic Riccati equation:

$$ A^T P + PA - PBR^{-1}B^T P + Q = 0 $$

where $P$ is the solution to the Riccati equation and $K = R^{-1}B^T P$.

## Contributing

Feel free to submit issues and pull requests for improvements.

## License

This work is licensed under a [Creative Commons Attribution 4.0 International License](http://creativecommons.org/licenses/by/4.0/). 

## Appendix A: Inverted Pendulum Dynamic Derivation

![Inverted Pendulum](doc/Cart-pendulum.svg.png)

### **System Dynamics**

We are given the following two second-order differential equations describing the dynamics of the system(in the python script):

```math
\begin{align*}
\ddot{x} &= \frac{F - m l \ddot{\theta} \cos(\theta) + m l \dot{\theta}^2 \sin(\theta)}{M + m} \quad \tag{1} \\
\ddot{\theta} &= \frac{g \sin(\theta) - \ddot{x} \cos(\theta)}{l} \quad \tag{2}
\end{align*}
```

Where:
- \( x \) is the position of mass \( M \).
- \( \theta \) is the angle of the pendulum.
- \( F \) is the external force applied to mass \( M \).
- \( g \) is the acceleration due to gravity.
- \( l \) is the length of the pendulum.
- \( m \) is the mass of the pendulum.

### **State Vector Definition**

We define the state vector as follows:

```math
\mathbf{State} = \mathbf{x} = \begin{bmatrix} x \\ \dot{x} \\ \theta \\ \dot{\theta} \end{bmatrix} = \begin{bmatrix} x_1 \\ x_2 \\ x_3 \\ x_4 \end{bmatrix}
```

Where:
- \( x_1 = x \)
- \( x_2 = \dot{x} \)
- \( x_3 = \theta \)
- \( x_4 = \dot{\theta} \)

The derivative of the state vector is:

```math
\dot{\mathbf{x}} = \begin{bmatrix} \dot{x}_1 \\ \dot{x}_2 \\ \dot{x}_3 \\ \dot{x}_4 \end{bmatrix} = \begin{bmatrix} x_2 \\ \ddot{x} \\ x_4 \\ \ddot{\theta} \end{bmatrix}
```

### **Step 1: Solve for \(\ddot{\theta}\) and Substitute into \(\ddot{x}\)**

First, substitute $(2)$ into equation $(1)$:

```math
\ddot{x} = \frac{F - m l \left( \frac{g \sin(\theta) - \ddot{x} \cos(\theta)}{l} \right) \cos(\theta) + m l \dot{\theta}^2 \sin(\theta)}{M + m}
```

Simplifying the above equation:

```math
\begin{align*}
\ddot{x} &= \frac{F - m \left( g \sin(\theta) - \ddot{x} \cos(\theta) \right) \cos(\theta) + m l \dot{\theta}^2 \sin(\theta)}{M + m} \\
&= \frac{F - m g \sin(\theta) \cos(\theta) + m \ddot{x} \cos^2(\theta) + m l \dot{\theta}^2 \sin(\theta)}{M + m}
\end{align*}
```

Rearranging terms to isolate \(\ddot{x}\):

```math
\ddot{x} (M + m) - m \ddot{x} \cos^2(\theta) = F - m g \sin(\theta) \cos(\theta) + m l \dot{\theta}^2 \sin(\theta)
```

Factor out \(\ddot{x}\):

```math
\ddot{x} \left( M + m - m \cos^2(\theta) \right) = F - m g \sin(\theta) \cos(\theta) + m l \dot{\theta}^2 \sin(\theta)
```

Finally, solve for \(\ddot{x}\):

```math
\ddot{x} = \frac{F - m g \sin(\theta) \cos(\theta) + m l \dot{\theta}^2 \sin(\theta)}{M + m - m \cos^2(\theta)} \quad \tag{3}
```

### **Step 2: Solve for \(\ddot{\theta}\)**

Using equation (2) and substituting the expression for \(\ddot{x}\) from equation (3):

```math
\ddot{\theta} = \frac{g \sin(\theta) - \ddot{x} \cos(\theta)}{l} = \frac{g \sin(\theta) - \left( \frac{F - m g \sin(\theta) \cos(\theta) + m l \dot{\theta}^2 \sin(\theta)}{M + m - m \cos^2(\theta)} \right) \cos(\theta)}{l}
```

Simplifying the numerator:

```math
\begin{align*}
\ddot{\theta} &= \frac{g \sin(\theta) (M + m - m \cos^2(\theta)) - F \cos(\theta) + m g \sin(\theta) \cos^2(\theta) - m l \dot{\theta}^2 \sin(\theta) \cos(\theta)}{l (M + m - m \cos^2(\theta))} \\
&= \frac{g (M + m) \sin(\theta) - F \cos(\theta) - m l \dot{\theta}^2 \sin(\theta) \cos(\theta)}{l (M + m - m \cos^2(\theta))} \quad \tag{4}
\end{align*}
```

### **Step 3: Formulate the State Space Model**

Substitute \(\ddot{x}\) and \(\ddot{\theta}\) into the state vector derivative:

```math
\dot{\mathbf{x}} = \begin{bmatrix} x_2 \\ \ddot{x} \\ x_4 \\ \ddot{\theta} \end{bmatrix} = \begin{bmatrix}
x_2 \\
\frac{F - m g \sin(x_3) \cos(x_3) + m l x_4^2 \sin(x_3)}{M + m - m \cos^2(x_3)} \\
x_4 \\
\frac{g (M + m) \sin(x_3) - F \cos(x_3) - m l x_4^2 \sin(x_3) \cos(x_3)}{l (M + m - m \cos^2(x_3))}
\end{bmatrix}
```

### **Step 4: Express in Matrix Form**

The state space model can be written as:

```math
\dot{\mathbf{x}} = \mathbf{A}(\mathbf{x}) \cdot \mathbf{x} + \mathbf{B}(\mathbf{x}) \cdot F
```

Where the system matrix \(\mathbf{A}(\mathbf{x})\) and input matrix \(\mathbf{B}(\mathbf{x})\) are defined as:

```math
\mathbf{A}(\mathbf{x}) =
\begin{bmatrix}
0 & 1 & 0 & 0 \\
0 & \frac{-m g \cos(x_3)}{M + m - m \cos^2(x_3)} & \frac{m l \sin(x_3)}{M + m - m \cos^2(x_3)} & \frac{2 m l x_4 \sin(x_3)}{M + m - m \cos^2(x_3)} \\
0 & 0 & 0 & 1 \\
0 & \frac{-\cos(x_3)}{l (M + m - m \cos^2(x_3))} & \frac{g (M + m) \cos(x_3)}{l (M + m - m \cos^2(x_3))} & \frac{-m l \sin(x_3) \cos(x_3)}{l (M + m - m \cos^2(x_3))}
\end{bmatrix}
```

```math
\mathbf{B}(\mathbf{x}) =
\begin{bmatrix}
0 \\
\frac{1}{M + m - m \cos^2(x_3)} \\
0 \\
\frac{-\cos(x_3)}{l (M + m - m \cos^2(x_3))}
\end{bmatrix}
```

### **Nonlinear State Space Model**

Putting it all together, the nonlinear state space model is:

```math
\dot{\mathbf{x}} =
\begin{bmatrix}
x_2 \\
\frac{F - m g \sin(x_3) \cos(x_3) + m l x_4^2 \sin(x_3)}{M + m - m \cos^2(x_3)} \\
x_4 \\
\frac{g (M + m) \sin(x_3) - F \cos(x_3) - m l x_4^2 \sin(x_3) \cos(x_3)}{l (M + m - m \cos^2(x_3))}
\end{bmatrix}
```

### **Step 5: Linearization (Small Angle Approximation)**

For control design, especially when dealing with small angles (\(\theta \approx 0\)), it is beneficial to linearize the nonlinear state space model. Using the small angle approximations:

```math
\sin(\theta) \approx \theta
```

```math
\cos(\theta) \approx 1
```

Substituting these approximations into equations (3) and (4):

```math
\ddot{x} \approx \frac{F - m g \theta}{M} \quad \text{(3 linearized)}
```

```math
\ddot{\theta} \approx \frac{g (M + m) \theta - F}{l M} \quad \text{(4 linearized)}
```

### **Linearized State Space Model**

The linearized state space model becomes:

```math
\dot{\mathbf{x}} =
\begin{bmatrix}
x_2 \\
\frac{F - m g x_3}{M} \\
x_4 \\
\frac{g (M + m) x_3 - F}{l M}
\end{bmatrix}
```

Expressed in matrix form:

```math
\dot{\mathbf{x}} = \mathbf{A}_{lin} \cdot \mathbf{x} + \mathbf{B}_{lin} \cdot F
```

Where the linearized system matrix \(\mathbf{A}_{lin}\) and input matrix \(\mathbf{B}_{lin}\) are:

```math
\mathbf{A}_{lin} =
\begin{bmatrix}
0 & 1 & 0 & 0 \\
0 & 0 & \frac{-m g}{M} & 0 \\
0 & 0 & 0 & 1 \\
0 & 0 & \frac{g (M + m)}{l M} & 0
\end{bmatrix}
```

```math
\mathbf{B}_{lin} =
\begin{bmatrix}
0 \\
\frac{1}{M} \\
0 \\
\frac{-1}{l M}
\end{bmatrix}
```
## Appendix 2: virutalenv on ROS2

- [Using Python Virtual Environments](https://github.com/ros2/ros2/issues/1094)
- Currently, there's a bug when using virtualenv with `--symlink-install`

## Appendix 3: humble 的 gazebo 行為

- 當執行 ros_gz_sim 的版本 < 7(預設是 6) -> `ign gazebo`
- 執行大於 7 -> `gz sim`
- 由於下載的是 binary 版本，ros_gz_bridge 等應該都只適配到 fortress?