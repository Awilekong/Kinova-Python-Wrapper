# 🤖 Kinova Gen3/G3L 机械臂 Python 高级控制 API

> 🚀 **高效、易用、专业的 Kinova 机械臂 Python 控制接口集合**  
> 适用于科研、教学、自动化开发、机器人竞赛等多种场景！

---

## 📦 项目简介

本项目基于 Kinova 官方 Python API，进行了二次封装，提供了更易用、更高层次的机械臂控制接口。  
涵盖了**笛卡尔空间运动、关节空间运动、轨迹规划、正逆运动学、状态查询、夹爪控制、阻抗控制**等完整功能，极大简化了二次开发流程。

---

## 🏗️ 目录结构

```
.
├── kinova_basic.py              # 🆕 Kinova机械臂高层控制类（推荐使用）
├── kinova_tool_configuration.py # 🆕 工具配置管理类
├── my_api_kinova.py             # 基础API脚本（函数式接口）
├── utilities.py                 # 连接与会话管理工具
├── franka_basic.py              # Franka机械臂控制类（参考实现）
├── Kinova-kortex2_Gen3_G3L/     # 官方SDK及示例
│   └── api_python/
│       └── ...                  # 官方原始API及示例
└── README.md                    # 项目说明文档
```

---

## ✨ 主要功能

### 🆕 **Kinova类（推荐使用）**
- **完整的机械臂控制类**：面向对象设计，易于使用和维护
- **夹爪控制**：支持Robotiq夹爪的精确控制（基于PyLibRM）
- **阻抗控制**：软/硬阻抗模式切换，支持柔顺控制
- **速度控制**：动态速度调整和关节速度限制
- **运动学计算**：正/逆运动学，支持工具配置
- **轨迹规划**：关节空间和笛卡尔空间轨迹执行
- **状态监控**：实时获取位置、速度、力矩、力传感器数据

### **基础API函数**
- **末端笛卡尔空间运动**（绝对/相对/轨迹）
- **关节空间运动**（单点/多点）
- **正运动学/逆运动学**（关节角↔末端位姿）
- **关节状态查询**（角度/力矩，支持角度制和弧度制）
- **动作完成/中止事件监听**
- **丰富的参数校验与异常处理**

---

## 🚀 快速上手

### 1. 安装依赖

```bash
# 安装Kinova官方SDK
pip install kortex_api-*.whl

# 安装夹爪控制库（可选）
pip install pylibrm
```

### 2. 使用Kinova类（推荐）

```python
from kinova_basic import Kinova

# 创建Kinova对象（包含夹爪）
kinova = Kinova(
    robot_ip="192.168.1.10",
    gripper_port="/dev/ttyUSB0"  # 夹爪串口
)

# 基本运动控制
kinova.set_ee_pose([0.5, 0.0, 0.3], [1, 0, 0, 0])  # 设置末端位姿
kinova.set_joint_pose([0, 0, 0, 0, 0, 0, 0])       # 设置关节角度

# 夹爪控制
kinova.open_gripper()    # 张开夹爪
kinova.close_gripper()   # 闭合夹爪
kinova.set_gripper_opening(50.0)  # 设置开口宽度50mm

# 阻抗控制
kinova.set_soft()        # 软阻抗模式
kinova.set_hard()        # 硬阻抗模式

# 状态查询
pose = kinova.get_ee_pose()      # 获取末端位姿
joints = kinova.get_joint_pose() # 获取关节角度
forces = kinova.get_ee_force()   # 获取末端力

# 关闭连接
kinova.close()
```

### 3. 工具配置管理

```python
from kinova_tool_configuration import KinovaToolConfiguration

# 创建工具配置管理器
tool_config = KinovaToolConfiguration(kinova)

# 设置自定义夹爪配置
custom_transform = {
    'x': 0.0,      # X方向偏移（米）
    'y': 0.0,      # Y方向偏移（米）
    'z': 0.080,    # Z方向偏移（米）
    'theta_x': 0.0, # X轴旋转（度）
    'theta_y': 0.0, # Y轴旋转（度）
    'theta_z': 0.0  # Z轴旋转（度）
}

tool_config.set_custom_gripper_config(
    transform_params=custom_transform,
    mass=0.3,  # 夹爪质量（千克）
    mass_center={'x': 0.0, 'y': 0.0, 'z': 0.040}
)
```

### 4. 使用基础API函数

```python
from my_api_kinova import *

# 连接设备
import utilities
args = utilities.parseConnectionArguments()
with utilities.DeviceConnection.createTcpConnection(args) as router:
    base = BaseClient(router)
    base_cyclic = BaseCyclicClient(router)

    # 末端相对移动
    cartesian_move(base, base_cyclic, dx=0.1, dy=0, dz=0)

    # 关节空间运动
    joint_move(base, [0, 0, 0, 0, 0, 0, 0])

    # 查询关节角（弧度）
    angles = get_joint_angles(base_cyclic, use_radian=True)
    print(angles)

    # 正运动学
    pose = compute_fk(base, angles)
    print(pose)
```

---

## 🛠️ API 速查表

### 🆕 **Kinova类方法**

| 功能类别 | 方法名 | 说明/参数 |
|---------|--------|-----------|
| **运动控制** | `set_ee_pose(ee_trans, ee_quat)` | 设置末端位姿（位置+四元数） |
| | `set_joint_pose(joint_pose)` | 设置关节角度 |
| | `cartesian_move(dx, dy, dz, ...)` | 末端相对运动 |
| | `set_joint_trajectories(trajectory)` | 执行关节轨迹 |
| **夹爪控制** | `open_gripper()` | 张开夹爪 |
| | `close_gripper()` | 闭合夹爪 |
| | `set_gripper_opening(width)` | 设置开口宽度 |
| | `gripper_precise_push(distance, force)` | 精密推压 |
| **阻抗控制** | `set_soft()` | 软阻抗模式 |
| | `set_hard()` | 硬阻抗模式 |
| | `start_impedance_control()` | 启动阻抗控制 |
| | `stop_impedance_control()` | 停止阻抗控制 |
| **速度控制** | `set_speed_factor(factor)` | 设置速度因子 |
| | `send_joint_speeds(speeds)` | 发送关节速度 |
| | `set_joint_velocity_limits(limits)` | 设置速度限制 |
| **状态查询** | `get_ee_pose()` | 获取末端位姿 |
| | `get_joint_pose()` | 获取关节角度 |
| | `get_ee_force()` | 获取末端力 |
| | `get_joint_torque()` | 获取关节力矩 |
| | `get_gripper_width()` | 获取夹爪宽度 |
| **运动学** | `compute_fk(joint_angles)` | 正运动学 |
| | `compute_ik(pose, guess)` | 逆运动学 |

### **基础API函数**

| 功能 | 函数名 | 说明/参数 |
|------|--------|-----------|
| 末端相对运动 | `cartesian_move` | dx, dy, dz, dtheta_x, dtheta_y, dtheta_z |
| 末端绝对运动 | `cartesian_set` | x, y, z, theta_x, theta_y, theta_z |
| 末端轨迹运动 | `cartesian_sequnence_move` | poses=[(x1,y1,z1,tx1,ty1,tz1), ...], blending_radius |
| 关节空间运动 | `joint_move` | joint_angles=[a1, a2, ..., a7] |
| 查询关节角 | `get_joint_angles` | use_radian=True/False |
| 查询关节力矩 | `get_joint_torque` | 无 |
| 正运动学 | `compute_fk` | joint_angles=[a1, ..., a7] |
| 逆运动学 | `compute_ik` | pose=(x, y, z, tx, ty, tz), guess=[a1, ..., a7]（可选） |

---

## 📖 完整示例

### 夹爪抓取示例

```python
from kinova_basic import Kinova

kinova = Kinova(robot_ip="192.168.1.10", gripper_port="/dev/ttyUSB0")

try:
    # 移动到抓取位置
    kinova.set_ee_pose([0.5, 0.0, 0.3], [1, 0, 0, 0])
    
    # 张开夹爪
    kinova.open_gripper(asynchronous=False)
    
    # 下降到物体位置
    kinova.cartesian_move(dz=-0.05)
    
    # 闭合夹爪抓取
    kinova.close_gripper(asynchronous=False)
    
    # 提起物体
    kinova.cartesian_move(dz=0.05)
    
    # 移动到放置位置
    kinova.set_ee_pose([0.5, 0.2, 0.3], [1, 0, 0, 0])
    
    # 张开夹爪释放
    kinova.open_gripper(asynchronous=False)
    
finally:
    kinova.close()
```

### 阻抗控制示例

```python
from kinova_basic import Kinova

kinova = Kinova(robot_ip="192.168.1.10")

try:
    # 设置软阻抗模式
    kinova.set_soft()
    
    # 启动阻抗控制
    kinova.start_impedance_control(stiffness=0.3, damping=0.1, duration=10)
    
    # 阻抗控制运行10秒
    import time
    time.sleep(10)
    
    # 停止阻抗控制
    kinova.stop_impedance_control()
    
finally:
    kinova.close()
```

---

## 🔧 工具配置指南

### 更换夹爪后的配置

当您更换夹爪后，必须更新工具配置以确保运动学计算准确：

1. **测量夹爪参数**：
   - 几何尺寸（安装面到夹爪中心的距离）
   - 质量
   - 质心位置

2. **设置工具配置**：
```python
tool_config = KinovaToolConfiguration(kinova)

# 设置自定义夹爪配置
custom_transform = {
    'x': 0.0,      # X方向偏移（米）
    'y': 0.0,      # Y方向偏移（米）
    'z': 0.080,    # Z方向偏移（米）- 关键参数
    'theta_x': 0.0, # X轴旋转（度）
    'theta_y': 0.0, # Y轴旋转（度）
    'theta_z': 0.0  # Z轴旋转（度）
}

tool_config.set_custom_gripper_config(
    transform_params=custom_transform,
    mass=0.3,  # 夹爪质量（千克）
    mass_center={'x': 0.0, 'y': 0.0, 'z': 0.040}
)
```

---

## 📝 注意事项

- **安全第一**：建议在安全环境下测试运动指令，避免机械臂碰撞
- **工具配置**：更换夹爪后必须更新工具配置，否则IK/FK计算不准确
- **夹爪连接**：确保夹爪串口连接正确，波特率设置匹配
- **阻抗控制**：阻抗控制模式下注意安全，避免意外碰撞
- **参数校验**：所有API均有参数校验，若输入不合法会抛出异常
- **轨迹规划**：轨迹运动建议路点数量适中，避免超时
- **逆运动学**：逆运动学解可能不唯一，初始猜测有助于收敛到期望解

---

## 🆕 新增功能说明

### **Kinova类特性**
- **面向对象设计**：更直观的API接口
- **自动连接管理**：自动处理连接建立和关闭
- **异常处理**：完善的错误处理和恢复机制
- **异步支持**：支持同步和异步执行模式
- **实时监控**：实时获取机械臂状态信息

### **夹爪控制功能**
- **精确控制**：支持毫米级精度的夹爪控制
- **力控制**：支持力限制和精密推压
- **状态监控**：实时获取夹爪宽度、力传感器数据
- **多种模式**：支持位置控制、力控制、推压模式

### **阻抗控制功能**
- **软/硬模式**：可切换的阻抗特性
- **实时控制**：1kHz控制频率的实时阻抗控制
- **参数调节**：可调节的刚度和阻尼参数
- **安全保护**：自动超时和错误恢复机制

---

## 💡 贡献与反馈

- 欢迎提交 Issue、PR 或建议，帮助本项目持续完善！
- 如需英文注释或国际化支持，请留言。
- 欢迎分享使用经验和改进建议！

---

## 📚 参考资料

- [Kinova 官方文档](https://github.com/Kinovarobotics/kortex)
- [Kinova Python API 示例](./Kinova-kortex2_Gen3_G3L/api_python/examples/)
- [PyLibRM 夹爪控制库](https://github.com/robotiq/pylibrm)

---

## 🏆 License

本项目基于 BSD 3-Clause License，详见 LICENSE 文件。

---

**让 Kinova 机械臂开发更高效、更优雅！**  
—— Powered by Python & Kinova 🤖✨

---
