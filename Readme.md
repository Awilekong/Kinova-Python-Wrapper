# 🤖 Kinova Gen3/G3L Python Advanced Control API

> 🚀 **A user-friendly, efficient, and professional Python interface for Kinova robotic arms**  
> Perfect for research, education, automation, and robotics competitions!

---

## 📦 Project Overview

This project is a high-level Python API wrapper based on the official Kinova Python SDK.  
It provides easy-to-use, high-level interfaces for controlling Kinova Gen3/G3L arms, including **Cartesian and joint space motion, trajectory planning, forward/inverse kinematics, state queries, gripper control, and impedance control**—making secondary development much easier.

---

## 🏗️ Directory Structure

```
.
├── kinova_basic.py              # 🆕 Kinova robot high-level control class (recommended)
├── kinova_tool_configuration.py # 🆕 Tool configuration management class
├── my_api_kinova.py             # Basic API script (functional interface)
├── utilities.py                 # Connection/session management utilities
├── franka_basic.py              # Franka robot control class (reference implementation)
├── Kinova-kortex2_Gen3_G3L/     # Official SDK and examples
│   └── api_python/
│       └── ...                  # Official API and demos
└── README.md                    # Project documentation
```

---

## ✨ Features

### 🆕 **Kinova Class (Recommended)**
- **Complete robot control class**: Object-oriented design, easy to use and maintain
- **Gripper control**: Precise control of Robotiq grippers (based on PyLibRM)
- **Impedance control**: Soft/hard impedance mode switching, compliant control support
- **Speed control**: Dynamic speed adjustment and joint speed limits
- **Kinematics computation**: Forward/inverse kinematics with tool configuration support
- **Trajectory planning**: Joint space and Cartesian space trajectory execution
- **State monitoring**: Real-time position, velocity, torque, and force sensor data

### **Basic API Functions**
- **Cartesian motion** (absolute, relative, trajectory)
- **Joint space motion** (single/multiple points)
- **Forward & inverse kinematics** (joint ↔ end-effector pose)
- **Joint state queries** (angles/torques, degree/radian support)
- **Action completion/abort event handling**
- **Robust parameter validation and exception handling**

---

## 🚀 Quick Start

### 1. Install Dependencies

```bash
# Install Kinova official SDK
pip install kortex_api-*.whl

# Install gripper control library (optional)
pip install pylibrm
```

### 2. Using Kinova Class (Recommended)

```python
from kinova_basic import Kinova

# Create Kinova object (with gripper)
kinova = Kinova(
    robot_ip="192.168.1.10",
    gripper_port="/dev/ttyUSB0"  # Gripper serial port
)

# Basic motion control
kinova.set_ee_pose([0.5, 0.0, 0.3], [1, 0, 0, 0])  # Set end-effector pose
kinova.set_joint_pose([0, 0, 0, 0, 0, 0, 0])       # Set joint angles

# Gripper control
kinova.open_gripper()    # Open gripper
kinova.close_gripper()   # Close gripper
kinova.set_gripper_opening(50.0)  # Set opening width to 50mm

# Impedance control
kinova.set_soft()        # Soft impedance mode
kinova.set_hard()        # Hard impedance mode

# State queries
pose = kinova.get_ee_pose()      # Get end-effector pose
joints = kinova.get_joint_pose() # Get joint angles
forces = kinova.get_ee_force()   # Get end-effector forces

# Close connection
kinova.close()
```

### 3. Tool Configuration Management

```python
from kinova_tool_configuration import KinovaToolConfiguration

# Create tool configuration manager
tool_config = KinovaToolConfiguration(kinova)

# Set custom gripper configuration
custom_transform = {
    'x': 0.0,      # X direction offset (meters)
    'y': 0.0,      # Y direction offset (meters)
    'z': 0.080,    # Z direction offset (meters)
    'theta_x': 0.0, # X axis rotation (degrees)
    'theta_y': 0.0, # Y axis rotation (degrees)
    'theta_z': 0.0  # Z axis rotation (degrees)
}

tool_config.set_custom_gripper_config(
    transform_params=custom_transform,
    mass=0.3,  # Gripper mass (kg)
    mass_center={'x': 0.0, 'y': 0.0, 'z': 0.040}
)
```

### 4. Using Basic API Functions

```python
from my_api_kinova import *

# Connect to the device
import utilities
args = utilities.parseConnectionArguments()
with utilities.DeviceConnection.createTcpConnection(args) as router:
    base = BaseClient(router)
    base_cyclic = BaseCyclicClient(router)

    # Cartesian relative move
    cartesian_move(base, base_cyclic, dx=0.1, dy=0, dz=0)

    # Joint space move
    joint_move(base, [0, 0, 0, 0, 0, 0, 0])

    # Query joint angles (radian)
    angles = get_joint_angles(base_cyclic, use_radian=True)
    print(angles)

    # Forward kinematics
    pose = compute_fk(base, angles)
    print(pose)
```

---

## 🛠️ API Quick Reference

### 🆕 **Kinova Class Methods**

| Category | Method | Description/Parameters |
|----------|--------|----------------------|
| **Motion Control** | `set_ee_pose(ee_trans, ee_quat)` | Set end-effector pose (position + quaternion) |
| | `set_joint_pose(joint_pose)` | Set joint angles |
| | `cartesian_move(dx, dy, dz, ...)` | End-effector relative motion |
| | `set_joint_trajectories(trajectory)` | Execute joint trajectory |
| **Gripper Control** | `open_gripper()` | Open gripper |
| | `close_gripper()` | Close gripper |
| | `set_gripper_opening(width)` | Set opening width |
| | `gripper_precise_push(distance, force)` | Precise pushing |
| **Impedance Control** | `set_soft()` | Soft impedance mode |
| | `set_hard()` | Hard impedance mode |
| | `start_impedance_control()` | Start impedance control |
| | `stop_impedance_control()` | Stop impedance control |
| **Speed Control** | `set_speed_factor(factor)` | Set speed factor |
| | `send_joint_speeds(speeds)` | Send joint speeds |
| | `set_joint_velocity_limits(limits)` | Set velocity limits |
| **State Queries** | `get_ee_pose()` | Get end-effector pose |
| | `get_joint_pose()` | Get joint angles |
| | `get_ee_force()` | Get end-effector forces |
| | `get_joint_torque()` | Get joint torques |
| | `get_gripper_width()` | Get gripper width |
| **Kinematics** | `compute_fk(joint_angles)` | Forward kinematics |
| | `compute_ik(pose, guess)` | Inverse kinematics |

### **Basic API Functions**

| Feature | Function Name | Description/Parameters |
|---------|---------------|----------------------|
| Cartesian relative move | `cartesian_move` | dx, dy, dz, dtheta_x, dtheta_y, dtheta_z |
| Cartesian absolute move | `cartesian_set` | x, y, z, theta_x, theta_y, theta_z |
| Cartesian trajectory | `cartesian_sequnence_move` | poses=[(x1,y1,z1,tx1,ty1,tz1), ...], blending_radius |
| Joint space move | `joint_move` | joint_angles=[a1, a2, ..., a7] |
| Query joint angles | `get_joint_angles` | use_radian=True/False |
| Query joint torques | `get_joint_torque` | None |
| Forward kinematics | `compute_fk` | joint_angles=[a1, ..., a7] |
| Inverse kinematics | `compute_ik` | pose=(x, y, z, tx, ty, tz), guess=[a1, ..., a7] (optional) |

---

## 📖 Complete Examples

### Gripper Grasping Example

```python
from kinova_basic import Kinova

kinova = Kinova(robot_ip="192.168.1.10", gripper_port="/dev/ttyUSB0")

try:
    # Move to grasping position
    kinova.set_ee_pose([0.5, 0.0, 0.3], [1, 0, 0, 0])
    
    # Open gripper
    kinova.open_gripper(asynchronous=False)
    
    # Lower to object position
    kinova.cartesian_move(dz=-0.05)
    
    # Close gripper to grasp
    kinova.close_gripper(asynchronous=False)
    
    # Lift object
    kinova.cartesian_move(dz=0.05)
    
    # Move to placement position
    kinova.set_ee_pose([0.5, 0.2, 0.3], [1, 0, 0, 0])
    
    # Open gripper to release
    kinova.open_gripper(asynchronous=False)
    
finally:
    kinova.close()
```

### Impedance Control Example

```python
from kinova_basic import Kinova

kinova = Kinova(robot_ip="192.168.1.10")

try:
    # Set soft impedance mode
    kinova.set_soft()
    
    # Start impedance control
    kinova.start_impedance_control(stiffness=0.3, damping=0.1, duration=10)
    
    # Run impedance control for 10 seconds
    import time
    time.sleep(10)
    
    # Stop impedance control
    kinova.stop_impedance_control()
    
finally:
    kinova.close()
```

---

## 🔧 Tool Configuration Guide

### Configuration After Gripper Replacement

When you replace the gripper, you must update the tool configuration to ensure accurate kinematics calculations:

1. **Measure gripper parameters**:
   - Geometric dimensions (distance from mounting surface to gripper center)
   - Mass
   - Center of mass position

2. **Set tool configuration**:
```python
tool_config = KinovaToolConfiguration(kinova)

# Set custom gripper configuration
custom_transform = {
    'x': 0.0,      # X direction offset (meters)
    'y': 0.0,      # Y direction offset (meters)
    'z': 0.080,    # Z direction offset (meters) - critical parameter
    'theta_x': 0.0, # X axis rotation (degrees)
    'theta_y': 0.0, # Y axis rotation (degrees)
    'theta_z': 0.0  # Z axis rotation (degrees)
}

tool_config.set_custom_gripper_config(
    transform_params=custom_transform,
    mass=0.3,  # Gripper mass (kg)
    mass_center={'x': 0.0, 'y': 0.0, 'z': 0.040}
)
```

---

## 📝 Notes

- **Safety first**: Test all motion commands in a safe environment to avoid collisions
- **Tool configuration**: Update tool configuration after gripper replacement, otherwise IK/FK calculations will be inaccurate
- **Gripper connection**: Ensure correct gripper serial port connection and matching baud rate
- **Impedance control**: Pay attention to safety during impedance control mode to avoid unexpected collisions
- **Parameter validation**: All APIs perform parameter validation and will raise exceptions for invalid input
- **Trajectory planning**: Keep the number of waypoints reasonable for trajectory motion to avoid timeouts
- **Inverse kinematics**: Inverse kinematics may have multiple solutions; providing an initial guess helps convergence

---

## 🆕 New Features

### **Kinova Class Features**
- **Object-oriented design**: More intuitive API interface
- **Automatic connection management**: Automatic connection establishment and cleanup
- **Exception handling**: Comprehensive error handling and recovery mechanisms
- **Async support**: Support for both synchronous and asynchronous execution modes
- **Real-time monitoring**: Real-time robot state information acquisition

### **Gripper Control Features**
- **Precise control**: Millimeter-level precision gripper control
- **Force control**: Force limiting and precise pushing support
- **State monitoring**: Real-time gripper width and force sensor data
- **Multiple modes**: Position control, force control, and pushing modes

### **Impedance Control Features**
- **Soft/hard modes**: Switchable impedance characteristics
- **Real-time control**: 1kHz control frequency real-time impedance control
- **Parameter adjustment**: Adjustable stiffness and damping parameters
- **Safety protection**: Automatic timeout and error recovery mechanisms

---

## 💡 Contributing & Feedback

- Issues, pull requests, and suggestions are welcome!
- For English/Chinese docstring support or other localization needs, please open an issue.
- Welcome to share usage experiences and improvement suggestions!

---

## 📚 References

- [Kinova Official Documentation](https://github.com/Kinovarobotics/kortex)
- [Kinova Python API Examples](./Kinova-kortex2_Gen3_G3L/api_python/examples/)
- [PyLibRM Gripper Control Library](https://github.com/robotiq/pylibrm)

---

## 🏆 License

This project is licensed under the BSD 3-Clause License. See the LICENSE file for details.

---

**Make Kinova robot development easier and more elegant!**  
—— Powered by Python & Kinova 🤖✨

---
