# 🤖 Kinova Gen3/G3L 机械臂 Python 二次封装 API

> 🚀 **高效、易用、专业的 Kinova 机械臂 Python 控制接口集合**  
> 适用于科研、教学、自动化开发、机器人竞赛等多种场景！

---

## 📦 项目简介

本项目基于 Kinova 官方 Python API，进行了二次封装，提供了更易用、更高层次的机械臂控制接口。  
涵盖了**笛卡尔空间运动、关节空间运动、轨迹规划、正逆运动学、状态查询**等常用功能，极大简化了二次开发流程。

---

## 🏗️ 目录结构

```
.
├── my_api_kinova.py         # 封装后的主API脚本（可直接import或运行）
├── utilities.py             # 连接与会话管理工具
├── Kinova-kortex2_Gen3_G3L/ # 官方SDK及示例
│   └── api_python/
│       └── ...              # 官方原始API及示例
└── README.md                # 项目说明文档
```

---

## ✨ 主要功能

- **末端笛卡尔空间运动**（绝对/相对/轨迹）
- **关节空间运动**（单点/多点）
- **正运动学/逆运动学**（关节角↔末端位姿）
- **关节状态查询**（角度/力矩，支持角度制和弧度制）
- **动作完成/中止事件监听**
- **丰富的参数校验与异常处理**
- **详细的中文注释与用法示例**

---

## 🚀 快速上手

### 1. 安装依赖

请先安装 Kinova 官方 Python SDK 及其依赖（如 protobuf、kortex_api 等）。

```bash
pip install kortex_api-*.whl
```

### 2. 连接机械臂

确保机械臂与PC网络互通，默认IP为 `192.168.1.10`，用户名/密码为 `admin/admin`。

### 3. 运行示例

```bash
python my_api_kinova.py
```

你可以在 `main()` 中根据注释，解开不同的示例代码，体验各类API功能。

---

## 🛠️ API 速查表

| 功能                      | 函数名                        | 说明/参数                                                                 |
|---------------------------|-------------------------------|--------------------------------------------------------------------------|
| 末端相对运动              | `cartesian_move`              | dx, dy, dz, dtheta_x, dtheta_y, dtheta_z                                 |
| 末端绝对运动              | `cartesian_set`               | x, y, z, theta_x, theta_y, theta_z                                       |
| 末端轨迹运动              | `cartesian_sequnence_move`    | poses=[(x1,y1,z1,tx1,ty1,tz1), ...], blending_radius                      |
| 关节空间运动              | `joint_move`                  | joint_angles=[a1, a2, ..., a7]                                           |
| 查询关节角                | `get_joint_angles`            | use_radian=True/False                                                    |
| 查询关节力矩              | `get_joint_torque`            | 无                                                                       |
| 正运动学                  | `compute_fk`                  | joint_angles=[a1, ..., a7]                                               |
| 逆运动学                  | `compute_ik`                  | pose=(x, y, z, tx, ty, tz), guess=[a1, ..., a7]（可选）                  |

> **详细参数、返回值、注意事项请见每个函数的 docstring 注释！**

---

## 📖 示例代码片段

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

## 📝 注意事项

- 建议在**安全环境**下测试运动指令，避免机械臂碰撞。
- 所有API均有参数校验，若输入不合法会抛出异常。
- 轨迹运动建议路点数量适中，避免超时。
- 逆运动学解可能不唯一，初始猜测有助于收敛到期望解。

---

## 💡 贡献与反馈

- 欢迎提交 Issue、PR 或建议，帮助本项目持续完善！
- 如需英文注释或国际化支持，请留言。

---

## 📚 参考资料

- [Kinova 官方文档](https://github.com/Kinovarobotics/kortex)
- [Kinova Python API 示例](./Kinova-kortex2_Gen3_G3L/api_python/examples/)

---

## 🏆 License

本项目基于 BSD 3-Clause License，详见 LICENSE 文件。

---

**让 Kinova 机械臂开发更高效、更优雅！**  
—— Powered by Python & Kinova 🤖✨

---
