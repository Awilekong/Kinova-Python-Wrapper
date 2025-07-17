# 🤖 Kinova Gen3/G3L Python Advanced Control API

> 🚀 **A user-friendly, efficient, and professional Python interface for Kinova robotic arms**  
> Perfect for research, education, automation, and robotics competitions!

---

## 📦 Project Overview

This project is a high-level Python API wrapper based on the official Kinova Python SDK.  
It provides easy-to-use, high-level interfaces for controlling Kinova Gen3/G3L arms, including **Cartesian and joint space motion, trajectory planning, forward/inverse kinematics, and state queries**—making secondary development much easier.

---

## 🏗️ Directory Structure

```
.
├── my_api_kinova.py         # Main API script (import or run directly)
├── utilities.py             # Connection/session management utilities
├── Kinova-kortex2_Gen3_G3L/ # Official SDK and examples
│   └── api_python/
│       └── ...              # Official API and demos
└── README.md                # Project documentation
```

---

## ✨ Features

- **Cartesian motion** (absolute, relative, trajectory)
- **Joint space motion** (single/multiple points)
- **Forward & inverse kinematics** (joint ↔ end-effector pose)
- **Joint state queries** (angles/torques, degree/radian support)
- **Action completion/abort event handling**
- **Robust parameter validation and exception handling**
- **Comprehensive docstrings and usage examples in English & Chinese**

---

## 🚀 Quick Start

### 1. Install Dependencies

First, install the Kinova official Python SDK and its dependencies (e.g., protobuf, kortex_api):

```bash
pip install kortex_api-*.whl
```

### 2. Connect to the Robot

Ensure your PC can reach the robot arm over the network.  
Default IP: `192.168.1.10`  
Default username/password: `admin/admin`

### 3. Run the Example

```bash
python my_api_kinova.py
```

Uncomment the relevant code blocks in `main()` to try out different API features.

---

## 🛠️ API Quick Reference

| Feature                   | Function Name                 | Description/Parameters                                                |
|---------------------------|-------------------------------|----------------------------------------------------------------------|
| Cartesian relative move   | `cartesian_move`              | dx, dy, dz, dtheta_x, dtheta_y, dtheta_z                             |
| Cartesian absolute move   | `cartesian_set`               | x, y, z, theta_x, theta_y, theta_z                                   |
| Cartesian trajectory      | `cartesian_sequnence_move`    | poses=[(x1,y1,z1,tx1,ty1,tz1), ...], blending_radius                  |
| Joint space move          | `joint_move`                  | joint_angles=[a1, a2, ..., a7]                                       |
| Query joint angles        | `get_joint_angles`            | use_radian=True/False                                                |
| Query joint torques       | `get_joint_torque`            | None                                                                 |
| Forward kinematics        | `compute_fk`                  | joint_angles=[a1, ..., a7]                                           |
| Inverse kinematics        | `compute_ik`                  | pose=(x, y, z, tx, ty, tz), guess=[a1, ..., a7] (optional)           |

> **See each function’s docstring for detailed parameters, return values, and usage notes!**

---

## 📖 Example Usage

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

## 📝 Notes

- **Test all motion commands in a safe environment** to avoid collisions.
- All APIs perform parameter validation and will raise exceptions for invalid input.
- For trajectory motion, keep the number of waypoints reasonable to avoid timeouts.
- Inverse kinematics may have multiple solutions; providing an initial guess helps convergence.

---

## 💡 Contributing & Feedback

- Issues, pull requests, and suggestions are welcome!
- For English/Chinese docstring support or other localization needs, please open an issue.

---

## 📚 References

- [Kinova Official Documentation](https://github.com/Kinovarobotics/kortex)
- [Kinova Python API Examples](./Kinova-kortex2_Gen3_G3L/api_python/examples/)

---

## 🏆 License

This project is licensed under the BSD 3-Clause License. See the LICENSE file for details.

---

**Make Kinova robot development easier and more elegant!**  
—— Powered by Python & Kinova 🤖✨

---
