#!/usr/bin/env python3

"""
kinova_basic.py
---------------
Kinova机械臂高层控制接口，封装了常用的运动、状态查询等功能。
参考franka_basic.py的结构设计，提供简洁易用的API。
"""

import sys
import os
import threading
import time
import numpy as np
from scipy.spatial.transform import Rotation
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.client_stubs.ActuatorConfigClientRpc import ActuatorConfigClient
from kortex_api.autogen.client_stubs.ActuatorCyclicClientRpc import ActuatorCyclicClient
from kortex_api.autogen.client_stubs.ControlConfigClientRpc import ControlConfigClient
from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, ActuatorConfig_pb2, ControlConfig_pb2
from kortex_api.RouterClient import RouterClientSendOptions


# 尝试导入PyLibRM夹爪库
try:
    from pylibrm import RMAxis
    GRIPPER_AVAILABLE = True
except ImportError:
    print("[Kinova] 警告: PyLibRM库未安装，夹爪功能将不可用")
    GRIPPER_AVAILABLE = False

# 默认动作等待超时时间（秒）
TIMEOUT_DURATION = 20


def check_for_end_or_abort(e):
    """
    动作完成/中止事件回调。
    用于阻塞等待机械臂动作结束。
    
    参数：
        e (threading.Event): 事件对象，动作完成/中止时被设置。
    返回：
        回调函数，供OnNotificationActionTopic订阅使用。
    """
    def check(notification, e = e):
        print("EVENT : " + Base_pb2.ActionEvent.Name(notification.action_event))
        if notification.action_event == Base_pb2.ACTION_END \
        or notification.action_event == Base_pb2.ACTION_ABORT:
            e.set()
    return check

class Kinova:
    """
    Kinova类：Kinova机械臂高层控制接口，封装了常用的运动、状态查询等功能。
    """
    
    def __init__(self, robot_ip="192.168.31.13", port=10000, relative_dynamics_factor=0.05, 
                 gripper_port="/dev/ttyUSB0", gripper_baudrate=115200, gripper_slave_id=1, use_curo_ik=False):
        """
        初始化Kinova机械臂对象。
        
        参数：
            robot_ip (str): 机器人IP地址，默认"192.168.1.10"
            port (int): 通信端口，默认10000
            relative_dynamics_factor (float): 运动学相对动态因子，影响速度/加速度等
            gripper_port (str): 夹爪串口路径，默认"/dev/ttyUSB0"
            gripper_baudrate (int): 夹爪波特率，默认115200
            gripper_slave_id (int): 夹爪从设备ID，默认1
        """
        # 导入utilities模块
        sys.path.insert(0, os.path.join(os.path.dirname(__file__), "Kinova-kortex2_Gen3_G3L/api_python/examples"))
        import utilities
        
        # 设置连接参数
        self.robot_ip = robot_ip
        self.port = port
        self.relative_dynamics_factor = relative_dynamics_factor
        
        # 创建连接参数
        self.args = utilities.parseConnectionArguments()
        self.args.ip = robot_ip
        self.args.port = port
        
        # 建立连接
        self.router = utilities.DeviceConnection.createTcpConnection(self.args)
        self.router = self.router.__enter__()  # 手动进入上下文管理器
        
        # 创建服务客户端
        self.base = BaseClient(self.router)
        self.base_cyclic = BaseCyclicClient(self.router)
        self.actuator_config = ActuatorConfigClient(self.router)
        self.control_config = ControlConfigClient(self.router)
        
        # 阻抗控制相关变量
        self.cyclic_running = False
        self.kill_the_thread = False
        self.cyclic_thread = None
        self.base_command = BaseCyclic_pb2.Command()
        self.base_feedback = BaseCyclic_pb2.Feedback()
        
        # 获取执行器数量
        self.actuator_count = self.base.GetActuatorCount().count
        
        # 初始化命令和反馈结构
        for i in range(self.actuator_count):
            self.base_command.actuators.add()
            self.base_feedback.actuators.add()
        
        # 设置发送选项
        self.sendOption = RouterClientSendOptions()
        self.sendOption.andForget = False
        self.sendOption.delay_ms = 0
        self.sendOption.timeout_ms = 3
        
        # 设置默认超时时间
        self.timeout_duration = TIMEOUT_DURATION
        
        # 初始化状态
        self.pose_shift = np.array([0, 0, 0])  # 末端位姿偏移
        self.start_joint_pose = [0, 0, 0, 0, 0, 0, 0]  # 预设起始关节角
        self.home_joint_pose = [0, 0, 0, 0, 0, 0, 0]   # 预设Home关节角
        
        # 初始化夹爪
        self.gripper = None
        self.gripper_port = gripper_port
        self.gripper_baudrate = gripper_baudrate
        self.gripper_slave_id = gripper_slave_id
        
        if GRIPPER_AVAILABLE:
            try:
                self.gripper = RMAxis.Axis_V6.create_modbus_rtu(
                    gripper_port, gripper_baudrate, gripper_slave_id
                )
                # 设置夹爪通信参数
                self.gripper.set_retries(5)
                self.gripper.set_timeout(100)  # 100ms超时
                # 开启夹爪伺服
                self.gripper.set_servo_on_off(True)
                # 设置默认运动参数
                self.gripper.config_motion(50.0, 100.0)  # 速度50mm/s, 加速度100mm/s²
                print(f"[Kinova] 夹爪连接成功: {gripper_port}")
            except Exception as e:
                print(f"[Kinova] 夹爪连接失败: {e}")
                self.gripper = None
        
        # 加载ikpy机械臂链
        urdf_path = os.path.join(os.path.dirname(__file__), "GEN3-7DOF-VISION_ARM_URDF_V12.urdf")
        # 自动替换continuous为revolute，生成临时URDF
        with open(urdf_path, "r") as f:
            urdf_str = f.read()
        urdf_str = urdf_str.replace('type="continuous"', 'type="revolute"')
        tmp_urdf_path = os.path.join(os.path.dirname(__file__), "gen3_tmp_ikpy.urdf")
        with open(tmp_urdf_path, "w") as f:
            f.write(urdf_str)
        self.chain = Chain.from_urdf_file(tmp_urdf_path, base_elements=["base_link"])
        print("[Kinova] ikpy机械臂链初始化成功")
        
        # 确保机械臂处于安全状态
        self.recover_from_errors()
        
        print(f"[Kinova] 成功连接到 {robot_ip}:{port}")

        if use_curo_ik:
            # Third Party
            import torch

            # cuRobo
            from curobo.types.base import TensorDeviceType
            from curobo.types.math import Pose
            from curobo.types.robot import RobotConfig
            from curobo.util_file import get_robot_configs_path, join_path, load_yaml, get_robot_path
            from curobo.wrap.reacher.ik_solver import IKSolver, IKSolverConfig
            from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel

            urdf_file = "/home/xi/pwei_space/curo/test/GEN3-7DOF-VISION_ARM_URDF_V12.urdf"
            base_link = "base_link"
            ee_link = "end_effector_link"
            tensor_args = TensorDeviceType()
            self.robot_cfg = RobotConfig.from_basic(urdf_file, base_link, ee_link, tensor_args)
            self.solver = IKSolver(self.robot_cfg)

    
    def curo_ik(self, position, quat, guess=None):
        """
        使用curobo的ik求解器求解逆运动学。
        
        参数：
            position (list/np.ndarray): 目标位置，格式为[x, y, z]
            quat (list/np.ndarray): 目标四元数，格式为[x, y, z, w]
            guess (list/np.ndarray): 初始猜测关节角（可选）  0-360
        返回：
            list/np.ndarray: 关节角 -pi~pi
        """
        quat = quat[[3,0,1,2]]
        if guess is None:
            guess = self.get_joint_pose()
            guess = [(q+180)%360-180 for q in guess]
            guess = np.deg2rad(guess)
        guess_tensor = torch.tensor(guess, dtype=torch.float32, device=self.tensor_args.device)
        goal_pose = Pose(torch.tensor(position, dtype=torch.float32, device=self.tensor_args.device), 
                         torch.tensor(quat, dtype=torch.float32, device=self.tensor_args.device))
        retract_config = torch.tensor(guess, dtype=torch.float32, device=self.tensor_args.device).unsqueeze(0)  # shape (1, 7)
        seed_config = torch.tensor(guess, dtype=torch.float32, device=self.tensor_args.device).unsqueeze(0).unsqueeze(0)  # shape (1, 1, 7)
        ik_result = self.solver.solve_single(
                    goal_pose,
                    retract_config=retract_config,    # 连续性正则化的参考配置
                    seed_config=seed_config,          # 用当前关节配置初始化所有 seeds
                    return_seeds=1,                   # 只返回误差最小的解
                    use_nn_seed=False
                )
        if ik_result.success[0].item():
            result = ik_result.solution[0].cpu().tolist()
        # 直接返回弧度，不进行角度转换
        # 验证结果 - 修复张量形状，处理嵌套列表
            if isinstance(result[0], list):
                result_flat = result[0]  # 提取嵌套列表
            else:
                result_flat = result
            return result_flat
        else:
            print("IK 求解失败")
            return None

    def curo_fk(self, joint_angles):
        """
        使用curobo的fk求解器求解正运动学。
        
        参数：
            joint_angles (list/np.ndarray): 关节角 0-360
        返回：
            ee_position (list/np.ndarray): 末端位置
            ee_quat (list/np.ndarray): 末端四元数 [x, y, z, w]
        """
        joint_angles = [(q+180)%360-180 for q in joint_angles]
        joint_angles = np.deg2rad(joint_angles)
        joint_angles_tensor = torch.tensor(joint_angles, dtype=torch.float32, device=self.tensor_args.device)
        fk_result = self.solver.fk(joint_angles_tensor)
        ee_position = fk_result.ee_position[0].cpu().tolist()
        ee_quat = fk_result.ee_quaternion[0].cpu().tolist()[[1,2,3,0]]
        return ee_position, ee_quat

    def ee_move_curo(self, position, quat, asynchronous=False):
        """
        使用curobo的fk求解器求解正运动学。
        
        参数：
            position (list/np.ndarray): 目标位置，格式为[x, y, z]
            quat (list/np.ndarray): 目标四元数，格式为[x, y, z, w]
            guess (list/np.ndarray): 初始猜测关节角（可选）  0-360
        返回：
            list/np.ndarray: 关节角
        """
        if not self.use_curo_ik:
            print("curobo ik 未启用，请在初始化时设置 use_curo_ik=True")
            return

        joint_angles = self.curo_ik(position, quat)
        if joint_angles is None:
            print("替换为默认笛卡尔控制器")
            self.ee_move(position, quat, asynchronous=asynchronous)
        else:
            joint_angles = [np.rad2deg(q) for q in joint_angles]
            joint_angles = [(q+360)%360 for q in joint_angles]
            self.set_joint_pose(joint_angles, asynchronous=asynchronous)

    def __del__(self):
        """析构函数，确保连接正确关闭。"""
        # 停止阻抗控制线程
        if hasattr(self, 'cyclic_running') and self.cyclic_running:
            self.stop_impedance_control()
        
        # 关闭夹爪连接
        if hasattr(self, 'gripper') and self.gripper is not None:
            try:
                self.gripper.close()
            except:
                pass
    
    def close(self):
        """关闭连接。"""
        # 停止阻抗控制线程
        if hasattr(self, 'cyclic_running') and self.cyclic_running:
            self.stop_impedance_control()
        
        # 关闭夹爪连接
        if hasattr(self, 'gripper') and self.gripper is not None:
            try:
                self.gripper.close()
            except:
                pass
        
        # 关闭机械臂连接
        if hasattr(self, 'router') and hasattr(self.router, '__exit__'):
            try:
                self.router.__exit__(None, None, None)
            except:
                pass
        
        print("[Kinova] 连接已关闭")
    
    def recover_from_errors(self):
        """确保机械臂处于无错误状态。"""
        try:
            # 设置单级伺服模式
            base_servo_mode = Base_pb2.ServoingModeInformation()
            base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
            self.base.SetServoingMode(base_servo_mode)
            print("[Kinova] 机械臂状态正常")
        except Exception as e:
            print(f"[Kinova] 恢复错误状态: {e}")
    
    def speed_down(self):
        """降低运动速度。"""
        self.relative_dynamics_factor *= 0.5
        print(f"[Kinova] 速度降低到 {self.relative_dynamics_factor}")
    
    def speed_normal(self):
        """恢复默认运动速度。"""
        self.relative_dynamics_factor = 0.05
        print(f"[Kinova] 速度恢复到 {self.relative_dynamics_factor}")
    
    def set_speed_factor(self, factor):
        """
        设置速度因子。
        
        参数：
            factor (float): 速度因子，0.1-1.0之间
        """
        if 0.1 <= factor <= 1.0:
            self.relative_dynamics_factor = factor
            print(f"[Kinova] 速度因子设置为 {factor}")
            return True
        else:
            print(f"[Kinova] 速度因子必须在0.1-1.0之间，当前值: {factor}")
            return False
    
    def set_default_pose(self):
        """
        让机器人回到预设的默认姿态。
        先到Home位，再到起始位。
        """
        print("[Kinova] 移动到默认姿态")
        self.set_joint_pose(self.home_joint_pose)
        self.set_joint_pose(self.start_joint_pose)
    
    def open_gripper(self, asynchronous=True):
        """
        张开夹爪。
        
        参数：
            asynchronous (bool): 是否异步执行
        返回：
            bool: True-动作完成，False-超时/中止
        """
        if self.gripper is None:
            print("[Kinova] 夹爪未连接")
            return False
        
        try:
            # 使用push方法张开夹爪，推压距离20mm，力限制15%
            self.gripper.push(20.0, 20.0, 500.0, 0.15, 0.1, 500.0)
            
            if not asynchronous:
                # 等待动作完成
                import time
                while not self.gripper.is_finished(15):
                    time.sleep(0.1)
                print("[Kinova] 夹爪张开完成")
                return True
            else:
                print("[Kinova] 夹爪张开指令已发送")
                return True
                
        except Exception as e:
            print(f"[Kinova] 夹爪张开失败: {e}")
            return False
    
    def close_gripper(self, asynchronous=True):
        """
        闭合夹爪。
        
        参数：
            asynchronous (bool): 是否异步执行
        返回：
            bool: True-动作完成，False-超时/中止
        """
        if self.gripper is None:
            print("[Kinova] 夹爪未连接")
            return False
        
        try:
            # 使用push方法闭合夹爪，推压距离20mm，力限制15%
            self.gripper.push(20.0, 20.0, 500.0, 0.15, 0.1, 500.0)
            
            if not asynchronous:
                # 等待动作完成
                import time
                while not self.gripper.is_finished(15):
                    time.sleep(0.1)
                print("[Kinova] 夹爪闭合完成")
                return True
            else:
                print("[Kinova] 夹爪闭合指令已发送")
                return True
                
        except Exception as e:
            print(f"[Kinova] 夹爪闭合失败: {e}")
            return False
    
    def set_gripper_opening(self, width, asynchronous=True):
        """
        设置夹爪开口宽度。
        
        参数：
            width (float): 目标开口宽度（毫米）
            asynchronous (bool): 是否异步执行
        返回：
            bool: True-动作完成，False-超时/中止
        """
        if self.gripper is None:
            print("[Kinova] 夹爪未连接")
            return False
        
        try:
            # 使用绝对运动到指定位置
            self.gripper.move_absolute(width, 50.0, 100.0, 100.0, 0.1)
            
            if not asynchronous:
                # 等待动作完成
                import time
                while not self.gripper.is_finished(15):
                    time.sleep(0.1)
                print(f"[Kinova] 夹爪宽度设置完成: {width}mm")
                return True
            else:
                print(f"[Kinova] 夹爪宽度设置指令已发送: {width}mm")
                return True
                
        except Exception as e:
            print(f"[Kinova] 夹爪宽度设置失败: {e}")
            return False
    

    
    def set_joint_pose(self, joint_pose, asynchronous=False):
        """
        控制机器人运动到指定关节角。
        
        参数：
            joint_pose (list/np.ndarray): 7维关节角
            asynchronous (bool): 是否异步执行
        """
        if len(joint_pose) != 7:
            raise ValueError("joint_pose 必须为长度为7的列表或数组")
        
        action = Base_pb2.Action()
        action.name = "Set Joint Pose"
        action.application_data = ""
        
        for joint_id, angle in enumerate(joint_pose):
            joint_angle = action.reach_joint_angles.joint_angles.joint_angles.add()
            joint_angle.joint_identifier = joint_id
            joint_angle.value = angle
        
        if not asynchronous:
            e = threading.Event()
            notification_handle = self.base.OnNotificationActionTopic(
                check_for_end_or_abort(e),
                Base_pb2.NotificationOptions()
            )
            
            print(f"[Kinova] 移动到关节角: {joint_pose}")
            self.base.ExecuteAction(action)
            
            finished = e.wait(self.timeout_duration)
            self.base.Unsubscribe(notification_handle)
            
            if finished:
                print("[Kinova] 关节姿态设置完成")
            else:
                print("[Kinova] 关节姿态设置超时或中止")
            
            return finished
        else:
            print(f"[Kinova] 异步移动到关节角: {joint_pose}")
            self.base.ExecuteAction(action)
            return True
    
    def set_joint_trajectories(self, joint_trajectory, velocity_trajectory=None, asynchronous=False):
        """
        执行关节空间轨迹。
        
        参数：
            joint_trajectory (np.ndarray): 关节角轨迹序列
            velocity_trajectory (np.ndarray): 关节速度轨迹序列（可选）
            asynchronous (bool): 是否异步执行
        """
        from kortex_api.autogen.messages import Base_pb2
        
        waypoints = Base_pb2.WaypointList()
        waypoints.duration = 0.0
        waypoints.use_optimal_blending = False
        
        for idx, joint_pose in enumerate(joint_trajectory):
            waypoint = waypoints.waypoints.add()
            waypoint.name = f"waypoint_{idx}"
            
            angular_wp = waypoint.angular_waypoint
            angular_wp.angles.extend(joint_pose)
            angular_wp.duration = 5.0  # 每个路点的持续时间
        
        # 验证轨迹
        result = self.base.ValidateWaypointList(waypoints)
        if len(result.trajectory_error_report.trajectory_error_elements) != 0:
            print("[Kinova] 关节轨迹验证失败")
            return False
        
        if not asynchronous:
            e = threading.Event()
            notification_handle = self.base.OnNotificationActionTopic(
                check_for_end_or_abort(e),
                Base_pb2.NotificationOptions()
            )
            
            print(f"[Kinova] 执行关节轨迹，路点数: {len(joint_trajectory)}")
            self.base.ExecuteWaypointTrajectory(waypoints)
            
            finished = e.wait(self.timeout_duration * len(joint_trajectory))
            self.base.Unsubscribe(notification_handle)
            
            if finished:
                print("[Kinova] 关节轨迹执行完成")
            else:
                print("[Kinova] 关节轨迹执行超时或中止")
            
            return finished
        else:
            print(f"[Kinova] 异步执行关节轨迹，路点数: {len(joint_trajectory)}")
            self.base.ExecuteWaypointTrajectory(waypoints)
            return True
    
    def set_ee_trajectory(self, waypoints, optimize=False, asynchronous=False):
        """
        执行笛卡尔空间路径点轨迹。
        
        参数：
            waypoints (list): 路径点列表，每个路径点为 (x, y, z, blending_radius, theta_x, theta_y, theta_z)
                             x, y, z: 位置（米）
                             blending_radius: 混合半径（米）
                             theta_x, theta_y, theta_z: 欧拉角（度，rxyz格式）
            asynchronous (bool): 是否异步执行
        返回：
            bool: True-动作完成，False-超时/中止
        """
        def populate_cartesian_coordinate(waypoint_information):
            """创建笛卡尔路径点对象"""
            waypoint = Base_pb2.CartesianWaypoint()
            waypoint.pose.x = waypoint_information[0]
            waypoint.pose.y = waypoint_information[1]
            waypoint.pose.z = waypoint_information[2]
            waypoint.blending_radius = waypoint_information[3]
            waypoint.pose.theta_x = waypoint_information[4]
            waypoint.pose.theta_y = waypoint_information[5]
            waypoint.pose.theta_z = waypoint_information[6]
            waypoint.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_BASE
            return waypoint
        
        try:
            # 确保机械臂处于单级伺服模式
            base_servo_mode = Base_pb2.ServoingModeInformation()
            base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
            self.base.SetServoingMode(base_servo_mode)
            
            # 创建路径点列表
            waypoints_list = Base_pb2.WaypointList()
            # 设置持续时间，单位为秒
            waypoints_list.duration = 0.0
            waypoints_list.use_optimal_blending = optimize
            
            # 添加路径点
            for i, waypoint_definition in enumerate(waypoints):
                waypoint = waypoints_list.waypoints.add()
                waypoint.name = f"waypoint_{i}"
                waypoint.cartesian_waypoint.CopyFrom(populate_cartesian_coordinate(waypoint_definition))
            
            # 验证路径点有效性
            result = self.base.ValidateWaypointList(waypoints_list)
            if len(result.trajectory_error_report.trajectory_error_elements) != 0:
                print("[Kinova] 路径点验证失败")
                result.trajectory_error_report.PrintDebugString()
                return False
            
            if not asynchronous:
                # 同步执行
                e = threading.Event()
                notification_handle = self.base.OnNotificationActionTopic(
                    check_for_end_or_abort(e),
                    Base_pb2.NotificationOptions()
                )
                
                print(f"[Kinova] 执行笛卡尔路径轨迹，路径点数: {len(waypoints)}")
                self.base.ExecuteWaypointTrajectory(waypoints_list)
                
                finished = e.wait()
                self.base.Unsubscribe(notification_handle)
                
                if finished:
                    print("[Kinova] 笛卡尔路径轨迹执行完成")
                else:
                    print("[Kinova] 笛卡尔路径轨迹执行超时或中止")
                
                return finished
            else:
                # 异步执行
                print(f"[Kinova] 异步执行笛卡尔路径轨迹，路径点数: {len(waypoints)}")
                self.base.ExecuteWaypointTrajectory(waypoints_list)
                return True
                
        except Exception as e:
            print(f"[Kinova] 笛卡尔路径轨迹执行失败: {e}")
            return False
    
    def set_ee_trajectory_with_optimization(self, waypoints, asynchronous=False):
        """
        执行优化的笛卡尔空间路径点轨迹。
        
        参数：
            waypoints (list): 路径点列表，每个路径点为 (x, y, z, blending_radius, theta_x, theta_y, theta_z)
            asynchronous (bool): 是否异步执行
        返回：
            bool: True-动作完成，False-超时/中止
        """
        def populate_cartesian_coordinate(waypoint_information):
            """创建笛卡尔路径点对象"""
            waypoint = Base_pb2.CartesianWaypoint()
            waypoint.pose.x = waypoint_information[0]
            waypoint.pose.y = waypoint_information[1]
            waypoint.pose.z = waypoint_information[2]
            waypoint.blending_radius = waypoint_information[3]
            waypoint.pose.theta_x = waypoint_information[4]
            waypoint.pose.theta_y = waypoint_information[5]
            waypoint.pose.theta_z = waypoint_information[6]
            waypoint.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_BASE
            return waypoint
        
        try:
            # 确保机械臂处于单级伺服模式
            base_servo_mode = Base_pb2.ServoingModeInformation()
            base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
            self.base.SetServoingMode(base_servo_mode)
            
            # 创建路径点列表（启用优化）
            waypoints_list = Base_pb2.WaypointList()
            waypoints_list.duration = 0.0
            waypoints_list.use_optimal_blending = True  # 启用优化
            
            # 添加路径点
            for i, waypoint_definition in enumerate(waypoints):
                waypoint = waypoints_list.waypoints.add()
                waypoint.name = f"waypoint_{i}"
                waypoint.cartesian_waypoint.CopyFrom(populate_cartesian_coordinate(waypoint_definition))
            
            # 验证路径点有效性
            result = self.base.ValidateWaypointList(waypoints_list)
            if len(result.trajectory_error_report.trajectory_error_elements) != 0:
                print("[Kinova] 优化路径点验证失败")
                result.trajectory_error_report.PrintDebugString()
                return False
            
            if not asynchronous:
                # 同步执行
                e = threading.Event()
                notification_handle = self.base.OnNotificationActionTopic(
                    check_for_end_or_abort(e),
                    Base_pb2.NotificationOptions()
                )
                
                print(f"[Kinova] 执行优化的笛卡尔路径轨迹，路径点数: {len(waypoints)}")
                self.base.ExecuteWaypointTrajectory(waypoints_list)
                
                finished = e.wait(self.timeout_duration)
                self.base.Unsubscribe(notification_handle)
                
                if finished:
                    print("[Kinova] 优化的笛卡尔路径轨迹执行完成")
                else:
                    print("[Kinova] 优化的笛卡尔路径轨迹执行超时或中止")
                
                return finished
            else:
                # 异步执行
                print(f"[Kinova] 异步执行优化的笛卡尔路径轨迹，路径点数: {len(waypoints)}")
                self.base.ExecuteWaypointTrajectory(waypoints_list)
                return True
                
        except Exception as e:
            print(f"[Kinova] 优化的笛卡尔路径轨迹执行失败: {e}")
            return False
    
    def get_ee_pose(self):
        """
        获取当前末端位姿（位置、四元数、欧拉角）。
        
        返回：
            shifted_ee_trans (np.ndarray): 末端位置
            ee_quat (np.ndarray): 末端四元数
            ee_rpy (np.ndarray): 末端欧拉角
        """
        feedback = self.base_cyclic.RefreshFeedback()
        
        # 获取末端位置
        ee_trans = np.array([
            feedback.base.tool_pose_x,
            feedback.base.tool_pose_y,
            feedback.base.tool_pose_z
        ])
        
        # 获取末端欧拉角并转换为四元数
        ee_euler_deg = np.array([
            feedback.base.tool_pose_theta_x,
            feedback.base.tool_pose_theta_y,
            feedback.base.tool_pose_theta_z
        ])
        ee_euler_rad = np.radians(ee_euler_deg)
        
        # 转换为四元数
        rotation = Rotation.from_euler('xyz', ee_euler_rad, degrees=False)
        ee_quat = rotation.as_quat()  # [x, y, z, w] 格式
        
        # 应用位姿偏移
        shifted_ee_trans = ee_trans + self.pose_shift
        
        return shifted_ee_trans, ee_quat, ee_euler_deg
    
    def get_joint_pose(self):
        """
        获取当前关节角度。
        
        返回：
            joint_pose (np.ndarray): 7维关节角
        """
        feedback = self.base_cyclic.RefreshFeedback()
        joint_pose = np.array([actuator.position for actuator in feedback.actuators])
        
        # 确保返回7维
        while len(joint_pose) < 7:
            joint_pose = np.append(joint_pose, 0.0)
        
        return joint_pose[:7]
    

    
    def get_joint_vel(self):
        """
        获取当前关节速度。
        
        返回：
            np.ndarray: 7维关节速度
        """
        feedback = self.base_cyclic.RefreshFeedback()
        joint_vel = np.array([actuator.velocity for actuator in feedback.actuators])
        
        # 确保返回7维
        while len(joint_vel) < 7:
            joint_vel = np.append(joint_vel, 0.0)
        
        return joint_vel[:7]
    
    def get_gripper_width(self):
        """
        获取当前夹爪开口宽度（毫米）。
        
        返回：
            float: 夹爪宽度（毫米）
        """
        if self.gripper is None:
            print("[Kinova] 夹爪未连接")
            return 0.0
        
        try:
            width = self.gripper.position()
            return width
        except Exception as e:
            print(f"[Kinova] 夹爪宽度获取失败: {e}")
            return 0.0
    
    def get_gripper_force(self):
        """
        获取夹爪力传感器数值（牛顿）。
        
        返回：
            float: 夹爪力传感器读数（N）
        """
        if self.gripper is None:
            print("[Kinova] 夹爪未连接")
            return 0.0
        
        try:
            force = self.gripper.force_sensor()
            return force
        except Exception as e:
            print(f"[Kinova] 夹爪力传感器读取失败: {e}")
            return 0.0
    
    def is_gripper_moving(self):
        """
        判断夹爪是否正在运动。
        
        返回：
            bool: 是否在运动
        """
        if self.gripper is None:
            return False
        
        try:
            return self.gripper.is_moving()
        except Exception as e:
            print(f"[Kinova] 夹爪运动状态检查失败: {e}")
            return False
    
    def is_gripper_captured(self):
        """
        判断夹爪是否夹持成功（用于推压运动）。
        
        返回：
            bool: 是否夹持成功
        """
        if self.gripper is None:
            return False
        
        try:
            return self.gripper.is_captured()
        except Exception as e:
            print(f"[Kinova] 夹爪夹持状态检查失败: {e}")
            return False
    
    def gripper_precise_push(self, distance, force, velocity_factor=1.0, impact_factor=0.0, band_n=0.1, band_ms=500.0):
        """
        夹爪精密推压。
        
        参数：
            distance (float): 推压距离（毫米）
            force (float): 目标力（牛顿）
            velocity_factor (float): 速度系数，默认1.0
            impact_factor (float): 冲击系数，默认0.0
            band_n (float): 力定位范围（牛顿），默认0.1
            band_ms (float): 稳压时间（毫秒），默认500.0
        返回：
            bool: True-动作完成，False-超时/中止
        """
        if self.gripper is None:
            print("[Kinova] 夹爪未连接")
            return False
        
        try:
            self.gripper.precise_push(distance, force, velocity_factor, impact_factor, band_n, band_ms)
            print(f"[Kinova] 夹爪精密推压: 距离={distance}mm, 目标力={force}N")
            return True
        except Exception as e:
            print(f"[Kinova] 夹爪精密推压失败: {e}")
            return False
    
    def gripper_go_home(self):
        """
        夹爪回原点。
        
        返回：
            bool: True-动作完成，False-超时/中止
        """
        if self.gripper is None:
            print("[Kinova] 夹爪未连接")
            return False
        
        try:
            self.gripper.go_home()
            print("[Kinova] 夹爪回原点指令已发送")
            return True
        except Exception as e:
            print(f"[Kinova] 夹爪回原点失败: {e}")
            return False
    
    def gripper_stop(self):
        """
        停止夹爪运动。
        """
        if self.gripper is None:
            print("[Kinova] 夹爪未连接")
            return
        
        try:
            self.gripper.stop()
            print("[Kinova] 夹爪停止运动")
        except Exception as e:
            print(f"[Kinova] 夹爪停止失败: {e}")
    
    def gripper_reset_error(self):
        """
        重置夹爪错误状态。
        """
        if self.gripper is None:
            print("[Kinova] 夹爪未连接")
            return
        
        try:
            self.gripper.reset_error()
            print("[Kinova] 夹爪错误状态已重置")
        except Exception as e:
            print(f"[Kinova] 夹爪错误重置失败: {e}")
    
    def get_ee_force(self):
        """
        获取末端受力（x/y/z方向分量及合力）。
        
        返回：
            fx, fy, fz (float): 末端受力分量
            normal_force (float): 合力
        """
        feedback = self.base_cyclic.RefreshFeedback()
        
        fx = feedback.base.tool_external_wrench_force_x
        fy = feedback.base.tool_external_wrench_force_y
        fz = feedback.base.tool_external_wrench_force_z
        
        normal_force = np.sqrt(fx**2 + fy**2 + fz**2)
        
        return fx, fy, fz, normal_force
    
    def get_joint_torque(self):
        """
        获取当前关节力矩。
        
        返回：
            np.ndarray: 7维关节力矩
        """
        feedback = self.base_cyclic.RefreshFeedback()
        joint_torque = np.array([actuator.torque for actuator in feedback.actuators])
        
        # 确保返回7维
        while len(joint_torque) < 7:
            joint_torque = np.append(joint_torque, 0.0)
        
        return joint_torque[:7]
    
    def compute_fk(self, joint_angles):
        """
        正运动学：根据关节角计算末端位姿。
        参数：
            joint_angles (list[float]): 长度为7的关节角度列表（度）。
        返回：
            tuple: (x, y, z, theta_x, theta_y, theta_z)
        """
        if not isinstance(joint_angles, (list, tuple)) or len(joint_angles) != 7:
            raise ValueError("joint_angles 必须为长度为7的列表或元组")
        from kortex_api.autogen.messages import Base_pb2
        joint_angles_msg = Base_pb2.JointAngles()
        for idx, angle in enumerate(joint_angles):
            joint_angle = joint_angles_msg.joint_angles.add()
            joint_angle.joint_identifier = idx
            joint_angle.value = angle
        pose = self.base.ComputeForwardKinematics(joint_angles_msg)
        return (pose.x, pose.y, pose.z, pose.theta_x, pose.theta_y, pose.theta_z)

    def compute_ik(self, pose, guess=None):
        """
        逆运动学：根据末端位姿计算关节角。
        参数：
            pose (tuple[float]): 末端位姿 (x, y, z, theta_x, theta_y, theta_z)
            guess (list[float]|None): 可选，长度为7的关节角初始猜测（度），有助于收敛。
        返回：
            list[float]: 七个关节角度列表（度）。
        """
        from kortex_api.autogen.messages import Base_pb2
        ik_data = Base_pb2.IKData()
        ik_data.cartesian_pose.x = pose[0]
        ik_data.cartesian_pose.y = pose[1]
        ik_data.cartesian_pose.z = pose[2]
        ik_data.cartesian_pose.theta_x = pose[3]
        ik_data.cartesian_pose.theta_y = pose[4]
        ik_data.cartesian_pose.theta_z = pose[5]
        if guess is not None and len(guess) == 7:
            for angle in guess:
                jAngle = ik_data.guess.joint_angles.add()
                jAngle.value = angle
        joint_angles_msg = self.base.ComputeInverseKinematics(ik_data)
        return [ja.value for ja in joint_angles_msg.joint_angles]

    def ik(self, pose, guess=None):
        """
        逆运动学：根据末端位姿计算关节角。
        欧拉角-180-180
        guess: 0-360
        输出0-360
        """
        from kortex_api.autogen.messages import Base_pb2
        input_IkData = Base_pb2.IKData()
        
        # Fill the IKData Object with the cartesian coordinates that need to be converted
        input_IkData.cartesian_pose.x = pose[0]
        input_IkData.cartesian_pose.y = pose[1]
        input_IkData.cartesian_pose.z = pose[2]
        input_IkData.cartesian_pose.theta_x = pose[3]
        input_IkData.cartesian_pose.theta_y = pose[4]
        input_IkData.cartesian_pose.theta_z = pose[5]

        # Fill the IKData Object with the guessed joint angles
        for joint_angle in guess:
            jAngle = input_IkData.guess.joint_angles.add()
            # '- 1' to generate an actual "guess" for current joint angles
            jAngle.value = joint_angle - 1
        print("Computing Inverse Kinematics using joint angles and pose...")
        computed_joint_angles = self.base.ComputeInverseKinematics(input_IkData)
        result = [ja.value % 360 for ja in computed_joint_angles.joint_angles]
        return result
    
    def cartesian_move(self, dx=0, dy=0, dz=0, dtheta_x=0, dtheta_y=0, dtheta_z=0):
        """
        末端笛卡尔空间相对运动。
        
        参数：
            dx, dy, dz (float): 末端在x/y/z方向的相对位移（单位：米）。
            dtheta_x, dtheta_y, dtheta_z (float): 末端绕x/y/z轴的相对旋转（单位：度）。
        返回：
            bool: True-动作完成，False-超时/中止。
        """
        feedback = self.base_cyclic.RefreshFeedback()
        
        action = Base_pb2.Action()
        action.name = "Cartesian Move"
        action.application_data = ""
        
        cartesian_pose = action.reach_pose.target_pose
        cartesian_pose.x = feedback.base.tool_pose_x + dx
        cartesian_pose.y = feedback.base.tool_pose_y + dy
        cartesian_pose.z = feedback.base.tool_pose_z + dz
        cartesian_pose.theta_x = feedback.base.tool_pose_theta_x + dtheta_x
        cartesian_pose.theta_y = feedback.base.tool_pose_theta_y + dtheta_y
        cartesian_pose.theta_z = feedback.base.tool_pose_theta_z + dtheta_z
        
        e = threading.Event()
        notification_handle = self.base.OnNotificationActionTopic(
            check_for_end_or_abort(e),
            Base_pb2.NotificationOptions()
        )
        
        print(f"[Kinova] 增量移动: dx={dx}, dy={dy}, dz={dz}, dtheta_x={dtheta_x}, dtheta_y={dtheta_y}, dtheta_z={dtheta_z}")
        self.base.ExecuteAction(action)
        
        finished = e.wait(self.timeout_duration)
        self.base.Unsubscribe(notification_handle)
        
        if finished:
            print("[Kinova] 增量移动完成")
        else:
            print("[Kinova] 增量移动超时或中止")
        
        return finished
    
    def start_impedance_control(self, stiffness=0.5, damping=0.1, duration=30):
        """
        启动阻抗控制模式。
        
        参数：
            stiffness (float): 刚度系数，默认0.5
            damping (float): 阻尼系数，默认0.1
            duration (int): 控制持续时间（秒），默认30秒，0表示无限
        返回：
            bool: 是否成功启动
        """
        if self.cyclic_running:
            print("[Kinova] 阻抗控制已在运行")
            return False
        
        try:
            # 初始化循环控制
            if not self._init_cyclic_control():
                return False
            
            # 设置阻抗参数
            self.impedance_stiffness = stiffness
            self.impedance_damping = damping
            self.cyclic_duration = duration
            
            # 启动循环线程
            self.cyclic_thread = threading.Thread(target=self._run_impedance_control)
            self.cyclic_thread.daemon = True
            self.cyclic_thread.start()
            
            print(f"[Kinova] 阻抗控制启动成功，刚度={stiffness}, 阻尼={damping}")
            return True
            
        except Exception as e:
            print(f"[Kinova] 阻抗控制启动失败: {e}")
            return False
    
    def stop_impedance_control(self):
        """
        停止阻抗控制模式。
        """
        if not self.cyclic_running:
            print("[Kinova] 阻抗控制未在运行")
            return
        
        try:
            # 停止循环线程
            self.kill_the_thread = True
            if self.cyclic_thread and self.cyclic_thread.is_alive():
                self.cyclic_thread.join(timeout=2.0)
            
            # 恢复位置控制模式
            self._restore_position_control()
            
            print("[Kinova] 阻抗控制已停止")
            
        except Exception as e:
            print(f"[Kinova] 阻抗控制停止失败: {e}")
    
    def _init_cyclic_control(self):
        """初始化循环控制。"""
        try:
            # 获取当前反馈
            base_feedback = self.base_cyclic.RefreshFeedback()
            if base_feedback:
                self.base_feedback = base_feedback
                
                # 初始化命令帧
                for i in range(self.actuator_count):
                    self.base_command.actuators[i].flags = 1  # servoing
                    self.base_command.actuators[i].position = self.base_feedback.actuators[i].position
                
                # 设置第一个执行器为力矩控制模式
                self.base_command.actuators[0].torque_joint = self.base_feedback.actuators[0].torque
                
                # 设置为低电平伺服模式
                base_servo_mode = Base_pb2.ServoingModeInformation()
                base_servo_mode.servoing_mode = Base_pb2.LOW_LEVEL_SERVOING
                self.base.SetServoingMode(base_servo_mode)
                
                # 发送第一帧
                self.base_feedback = self.base_cyclic.Refresh(self.base_command, 0, self.sendOption)
                
                # 设置第一个执行器为力矩模式
                control_mode_message = ActuatorConfig_pb2.ControlModeInformation()
                control_mode_message.control_mode = ActuatorConfig_pb2.ControlMode.Value('TORQUE')
                device_id = 1  # 第一个执行器ID为1
                
                self.actuator_config.SetControlMode(control_mode_message, device_id)
                
                return True
            else:
                print("[Kinova] 初始化循环控制失败：无法获取反馈")
                return False
                
        except Exception as e:
            print(f"[Kinova] 初始化循环控制失败: {e}")
            return False
    
    def _run_impedance_control(self):
        """运行阻抗控制循环。"""
        self.cyclic_running = True
        print("[Kinova] 阻抗控制循环开始")
        
        cyclic_count = 0
        failed_cyclic_count = 0
        t_init = time.time()
        
        while not self.kill_the_thread:
            try:
                # 阻抗控制逻辑
                for i in range(self.actuator_count):
                    # 计算期望力矩（简化的阻抗控制）
                    position_error = self.base_command.actuators[i].position - self.base_feedback.actuators[i].position
                    velocity = self.base_feedback.actuators[i].velocity
                    
                    # 阻抗控制公式：τ = K*e + D*ẋ
                    desired_torque = (self.impedance_stiffness * position_error - 
                                    self.impedance_damping * velocity)
                    
                    if i == 0:  # 第一个执行器使用力矩控制
                        self.base_command.actuators[i].torque_joint = desired_torque
                    else:  # 其他执行器保持位置控制
                        self.base_command.actuators[i].position = self.base_feedback.actuators[i].position
                
                # 更新帧ID
                self.base_command.frame_id += 1
                if self.base_command.frame_id > 65535:
                    self.base_command.frame_id = 0
                
                for i in range(self.actuator_count):
                    self.base_command.actuators[i].command_id = self.base_command.frame_id
                
                # 发送命令帧
                self.base_feedback = self.base_cyclic.Refresh(self.base_command, 0, self.sendOption)
                cyclic_count += 1
                
                # 检查持续时间
                if self.cyclic_duration > 0 and (time.time() - t_init) > self.cyclic_duration:
                    print("[Kinova] 阻抗控制达到指定持续时间")
                    break
                
                # 控制循环频率
                time.sleep(0.001)  # 1kHz控制频率
                
            except Exception as e:
                failed_cyclic_count += 1
                if failed_cyclic_count > 10:
                    print(f"[Kinova] 阻抗控制循环错误过多: {e}")
                    break
                time.sleep(0.001)
        
        self.cyclic_running = False
        print("[Kinova] 阻抗控制循环结束")
    
    def _restore_position_control(self):
        """恢复位置控制模式。"""
        try:
            # 设置第一个执行器回到位置控制模式
            control_mode_message = ActuatorConfig_pb2.ControlModeInformation()
            control_mode_message.control_mode = ActuatorConfig_pb2.ControlMode.Value('POSITION')
            device_id = 1  # 第一个执行器ID为1
            self.actuator_config.SetControlMode(control_mode_message, device_id)
            
            # 恢复单级伺服模式
            base_servo_mode = Base_pb2.ServoingModeInformation()
            base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
            self.base.SetServoingMode(base_servo_mode)
            
        except Exception as e:
            print(f"[Kinova] 恢复位置控制模式失败: {e}")
    
    def send_joint_speeds(self, joint_speeds, duration=0):
        """
        发送关节速度命令。
        
        参数：
            joint_speeds (list): 关节速度列表（度/秒）
            duration (float): 持续时间（秒），0表示持续到停止
        返回：
            bool: 是否成功发送
        """
        try:
            if len(joint_speeds) != self.actuator_count:
                print(f"[Kinova] 关节速度数量必须为{self.actuator_count}")
                return False
            
            # 创建关节速度命令
            joint_speeds_msg = Base_pb2.JointSpeeds()
            
            for i, speed in enumerate(joint_speeds):
                joint_speed = joint_speeds_msg.joint_speeds.add()
                joint_speed.joint_identifier = i
                joint_speed.value = speed
                # duration字段在某些版本中可能不可用，使用try-except处理
                try:
                    joint_speed.duration = duration
                except AttributeError:
                    # 如果duration字段不存在，忽略它
                    pass
            
            # 发送速度命令
            self.base.SendJointSpeedsCommand(joint_speeds_msg)
            print(f"[Kinova] 关节速度命令已发送: {joint_speeds}")
            return True
            
        except Exception as e:
            print(f"[Kinova] 关节速度命令发送失败: {e}")
            return False
    
    def stop_motion(self):
        """
        停止机械臂运动。
        """
        try:
            self.base.Stop()
            print("[Kinova] 机械臂运动已停止")
            return True
        except Exception as e:
            print(f"[Kinova] 停止机械臂运动失败: {e}")
            return False

    def ee_move(self, trans, quat, asynchronous=False):
        R = Rotation.from_quat(quat)
        euler = R.as_euler('xyz', degrees=True)
        action = Base_pb2.Action()
        action.name = "Example Cartesian action movement"
        action.application_data = ""

        cartesian_pose = action.reach_pose.target_pose
        cartesian_pose.x = trans[0]        # (meters)
        cartesian_pose.y = trans[1]    # (meters)
        cartesian_pose.z = trans[2]    # (meters)
        cartesian_pose.theta_x = euler[0] # (degrees)
        cartesian_pose.theta_y = euler[1] # (degrees)
        cartesian_pose.theta_z = euler[2] # (degrees)

        e = threading.Event()
        def concise_check(notification, e=e):
            event_name = Base_pb2.ActionEvent.Name(notification.action_event)
            if notification.action_event == Base_pb2.ACTION_ABORT:
                print(f"EVENT : {event_name}  (奇异位置/无法到达/超限)")
            else:
                print(f"EVENT : {event_name}")
            if notification.action_event == Base_pb2.ACTION_END \
            or notification.action_event == Base_pb2.ACTION_ABORT:
                e.set()

        notification_handle = self.base.OnNotificationActionTopic(
            concise_check,
            Base_pb2.NotificationOptions()
        )
        if asynchronous:
            self.base.ExecuteAction(action)
            return True
        else:
            self.base.ExecuteAction(action)

            finished = e.wait(self.timeout_duration)
            self.base.Unsubscribe(notification_handle)

            return finished

    def set_joint_speed_limits(self, speed_limit, control_mode=None):
        """
        设置关节速度软限制。
        
        参数：
            speed_limit (float): 关节速度限制（度/秒）
            control_mode (int): 控制模式，默认为ANGULAR_TRAJECTORY
        返回：
            bool: 是否成功设置
        """
        try:
            if control_mode is None:
                control_mode = ControlConfig_pb2.ControlMode.Value('ANGULAR_TRAJECTORY')
            
            joint_speed_limits = ControlConfig_pb2.JointSpeedSoftLimits()
            joint_speed_limits.control_mode = control_mode
            joint_speed_limits.joint_speed_soft_limits = speed_limit
            
            self.control_config.SetJointSpeedSoftLimits(joint_speed_limits)
            print(f"[Kinova] 关节速度限制设置为 {speed_limit} 度/秒")
            return True
            
        except Exception as e:
            print(f"[Kinova] 设置关节速度限制失败: {e}")
            return False
    
    def get_joint_speed_limits(self, control_mode=None):
        """
        获取当前关节速度软限制。
        
        参数：
            control_mode (int): 控制模式，默认为ANGULAR_TRAJECTORY
        返回：
            float: 关节速度限制（度/秒），失败返回None
        """
        try:
            if control_mode is None:
                control_mode = ControlConfig_pb2.ControlMode.Value('ANGULAR_TRAJECTORY')
            
            joint_speed_limits = ControlConfig_pb2.JointSpeedSoftLimits()
            joint_speed_limits.control_mode = control_mode
            
            result = self.control_config.GetJointSpeedSoftLimits(joint_speed_limits)
            return result.joint_speed_soft_limits
            
        except Exception as e:
            print(f"[Kinova] 获取关节速度限制失败: {e}")
            return None
    
    def increase_joint_speed_limits(self, increment=10.0):
        """
        增大关节速度限制。
        
        参数：
            increment (float): 增大量（度/秒），默认10.0
        返回：
            bool: 是否成功增大
        """
        try:
            current_limit = self.get_joint_speed_limits()
            if current_limit is None:
                print("[Kinova] 无法获取当前关节速度限制")
                return False
            
            new_limit = current_limit + increment
            success = self.set_joint_speed_limits(new_limit)
            if success:
                print(f"[Kinova] 关节速度限制从 {current_limit} 增大到 {new_limit} 度/秒")
            return success
            
        except Exception as e:
            print(f"[Kinova] 增大关节速度限制失败: {e}")
            return False
    
    def decrease_joint_speed_limits(self, decrement=10.0):
        """
        减小关节速度限制。
        
        参数：
            decrement (float): 减小量（度/秒），默认10.0
        返回：
            bool: 是否成功减小
        """
        try:
            current_limit = self.get_joint_speed_limits()
            if current_limit is None:
                print("[Kinova] 无法获取当前关节速度限制")
                return False
            
            new_limit = max(0.0, current_limit - decrement)  # 确保不为负数
            success = self.set_joint_speed_limits(new_limit)
            if success:
                print(f"[Kinova] 关节速度限制从 {current_limit} 减小到 {new_limit} 度/秒")
            return success
            
        except Exception as e:
            print(f"[Kinova] 减小关节速度限制失败: {e}")
            return False
    
    def set_cartesian_linear_speed_limit(self, speed_limit, control_mode=None):
        """
        设置笛卡尔线性速度软限制。
        
        参数：
            speed_limit (float): 线性速度限制（米/秒）
            control_mode (int): 控制模式，默认为CARTESIAN_TRAJECTORY
        返回：
            bool: 是否成功设置
        """
        try:
            if control_mode is None:
                control_mode = ControlConfig_pb2.ControlMode.Value('CARTESIAN_TRAJECTORY')
            
            twist_linear_limit = ControlConfig_pb2.TwistLinearSoftLimit()
            twist_linear_limit.control_mode = control_mode
            twist_linear_limit.twist_linear_soft_limit = speed_limit
            
            self.control_config.SetTwistLinearSoftLimit(twist_linear_limit)
            print(f"[Kinova] 笛卡尔线性速度限制设置为 {speed_limit} 米/秒")
            return True
            
        except Exception as e:
            print(f"[Kinova] 设置笛卡尔线性速度限制失败: {e}")
            return False
    
    def get_cartesian_linear_speed_limit(self, control_mode=None):
        """
        获取当前笛卡尔线性速度软限制。
        
        参数：
            control_mode (int): 控制模式，默认为CARTESIAN_TRAJECTORY
        返回：
            float: 线性速度限制（米/秒），失败返回None
        """
        try:
            if control_mode is None:
                control_mode = ControlConfig_pb2.ControlMode.Value('CARTESIAN_TRAJECTORY')
            
            twist_linear_limit = ControlConfig_pb2.TwistLinearSoftLimit()
            twist_linear_limit.control_mode = control_mode
            
            result = self.control_config.GetTwistLinearSoftLimit(twist_linear_limit)
            return result.twist_linear_soft_limit
            
        except Exception as e:
            print(f"[Kinova] 获取笛卡尔线性速度限制失败: {e}")
            return None
    
    def increase_cartesian_linear_speed_limit(self, increment=0.1):
        """
        增大笛卡尔线性速度限制。
        
        参数：
            increment (float): 增大量（米/秒），默认0.1
        返回：
            bool: 是否成功增大
        """
        try:
            current_limit = self.get_cartesian_linear_speed_limit()
            if current_limit is None:
                print("[Kinova] 无法获取当前笛卡尔线性速度限制")
                return False
            
            new_limit = current_limit + increment
            success = self.set_cartesian_linear_speed_limit(new_limit)
            if success:
                print(f"[Kinova] 笛卡尔线性速度限制从 {current_limit} 增大到 {new_limit} 米/秒")
            return success
            
        except Exception as e:
            print(f"[Kinova] 增大笛卡尔线性速度限制失败: {e}")
            return False
    
    def decrease_cartesian_linear_speed_limit(self, decrement=0.1):
        """
        减小笛卡尔线性速度限制。
        
        参数：
            decrement (float): 减小量（米/秒），默认0.1
        返回：
            bool: 是否成功减小
        """
        try:
            current_limit = self.get_cartesian_linear_speed_limit()
            if current_limit is None:
                print("[Kinova] 无法获取当前笛卡尔线性速度限制")
                return False
            
            new_limit = max(0.0, current_limit - decrement)  # 确保不为负数
            success = self.set_cartesian_linear_speed_limit(new_limit)
            if success:
                print(f"[Kinova] 笛卡尔线性速度限制从 {current_limit} 减小到 {new_limit} 米/秒")
            return success
            
        except Exception as e:
            print(f"[Kinova] 减小笛卡尔线性速度限制失败: {e}")
            return False
    
    def set_cartesian_angular_speed_limit(self, speed_limit, control_mode=None):
        """
        设置笛卡尔角速度软限制。
        
        参数：
            speed_limit (float): 角速度限制（度/秒）
            control_mode (int): 控制模式，默认为CARTESIAN_TRAJECTORY
        返回：
            bool: 是否成功设置
        """
        try:
            if control_mode is None:
                control_mode = ControlConfig_pb2.ControlMode.Value('CARTESIAN_TRAJECTORY')
            
            twist_angular_limit = ControlConfig_pb2.TwistAngularSoftLimit()
            twist_angular_limit.control_mode = control_mode
            twist_angular_limit.twist_angular_soft_limit = speed_limit
            
            self.control_config.SetTwistAngularSoftLimit(twist_angular_limit)
            print(f"[Kinova] 笛卡尔角速度限制设置为 {speed_limit} 度/秒")
            return True
            
        except Exception as e:
            print(f"[Kinova] 设置笛卡尔角速度限制失败: {e}")
            return False
    
    def get_cartesian_angular_speed_limit(self, control_mode=None):
        """
        获取当前笛卡尔角速度软限制。
        
        参数：
            control_mode (int): 控制模式，默认为CARTESIAN_TRAJECTORY
        返回：
            float: 角速度限制（度/秒），失败返回None
        """
        try:
            if control_mode is None:
                control_mode = ControlConfig_pb2.ControlMode.Value('CARTESIAN_TRAJECTORY')
            
            twist_angular_limit = ControlConfig_pb2.TwistAngularSoftLimit()
            twist_angular_limit.control_mode = control_mode
            
            result = self.control_config.GetTwistAngularSoftLimit(twist_angular_limit)
            return result.twist_angular_soft_limit
            
        except Exception as e:
            print(f"[Kinova] 获取笛卡尔角速度限制失败: {e}")
            return None
    
    def increase_cartesian_angular_speed_limit(self, increment=10.0):
        """
        增大笛卡尔角速度限制。
        
        参数：
            increment (float): 增大量（度/秒），默认10.0
        返回：
            bool: 是否成功增大
        """
        try:
            current_limit = self.get_cartesian_angular_speed_limit()
            if current_limit is None:
                print("[Kinova] 无法获取当前笛卡尔角速度限制")
                return False
            
            new_limit = current_limit + increment
            success = self.set_cartesian_angular_speed_limit(new_limit)
            if success:
                print(f"[Kinova] 笛卡尔角速度限制从 {current_limit} 增大到 {new_limit} 度/秒")
            return success
            
        except Exception as e:
            print(f"[Kinova] 增大笛卡尔角速度限制失败: {e}")
            return False
    
    def decrease_cartesian_angular_speed_limit(self, decrement=10.0):
        """
        减小笛卡尔角速度限制。
        
        参数：
            decrement (float): 减小量（度/秒），默认10.0
        返回：
            bool: 是否成功减小
        """
        try:
            current_limit = self.get_cartesian_angular_speed_limit()
            if current_limit is None:
                print("[Kinova] 无法获取当前笛卡尔角速度限制")
                return False
            
            new_limit = max(0.0, current_limit - decrement)  # 确保不为负数
            success = self.set_cartesian_angular_speed_limit(new_limit)
            if success:
                print(f"[Kinova] 笛卡尔角速度限制从 {current_limit} 减小到 {new_limit} 度/秒")
            return success
            
        except Exception as e:
            print(f"[Kinova] 减小笛卡尔角速度限制失败: {e}")
            return False
    
    def print_speed_limits(self):
        """
        打印当前所有速度限制信息。
        """
        print("\n=== 当前速度限制信息 ===")
        
        # 关节速度限制
        joint_limit = self.get_joint_speed_limits()
        if joint_limit is not None:
            print(f"关节速度限制: {joint_limit} 度/秒")
        else:
            print("关节速度限制: 获取失败")
        
        # 笛卡尔线性速度限制
        linear_limit = self.get_cartesian_linear_speed_limit()
        if linear_limit is not None:
            print(f"笛卡尔线性速度限制: {linear_limit} 米/秒")
        else:
            print("笛卡尔线性速度限制: 获取失败")
        
        # 笛卡尔角速度限制
        angular_limit = self.get_cartesian_angular_speed_limit()
        if angular_limit is not None:
            print(f"笛卡尔角速度限制: {angular_limit} 度/秒")
        else:
            print("笛卡尔角速度限制: 获取失败")
        
        print("========================\n")


# 使用示例
if __name__ == "__main__":
    # 创建Kinova对象（包含夹爪）
    kinova = None
    try:
        kinova = Kinova(robot_ip="192.168.31.13", gripper_port="/dev/ttyUSB0")
        
        # 获取当前状态
        current_pose = kinova.get_joint_pose()
        print(f"当前关节角: {current_pose}")
        
        ee_pos, ee_quat, ee_euler = kinova.get_ee_pose()
        print(f"当前末端位置: {ee_pos}")
        print(f"当前末端四元数: {ee_quat}")
        print(f"当前末端欧拉角: {np.degrees(ee_euler)}")
        
        # 获取力和力矩信息
        fx, fy, fz, total_force = kinova.get_ee_force()
        print(f"末端力: [{fx:.2f}, {fy:.2f}, {fz:.2f}], 合力: {total_force:.2f}N")
        
        joint_torques = kinova.get_joint_torque()
        print(f"关节力矩: {joint_torques}")
        
        joint_velocities = kinova.get_joint_vel()
        print(f"关节速度: {joint_velocities}")
        
        # 夹爪相关测试
        if kinova.gripper is not None:
            print("\n=== 夹爪测试 ===")
            
            # 获取夹爪状态
            gripper_width = kinova.get_gripper_width()
            gripper_force = kinova.get_gripper_force()
            print(f"夹爪宽度: {gripper_width:.2f} mm")
            print(f"夹爪力: {gripper_force:.2f} N")
            
            # 测试夹爪开合
            print("测试夹爪张开...")
            kinova.open_gripper(asynchronous=False)
            
            print("测试夹爪闭合...")
            kinova.close_gripper(asynchronous=False)
            
            # 测试精密推压
            print("测试精密推压...")
            kinova.gripper_precise_push(10.0, 5.0)  # 推压10mm，目标力5N
            
        else:
            print("夹爪未连接，跳过夹爪测试")
        
        # 阻抗控制和速度调整测试
        print("\n=== 阻抗控制和速度调整测试 ===")
        
        # 测试速度调整
        print("测试速度调整...")
        kinova.set_speed_factor(0.3)  # 降低速度到30%
        kinova.speed_normal()  # 恢复正常速度
        
        # 测试关节速度命令（仅发送命令，不实际执行）
        print("测试关节速度命令...")
        test_speeds = [10.0, 0.0, -10.0, 0.0, 10.0, 0.0, -10.0]  # 测试速度
        kinova.send_joint_speeds(test_speeds, duration=0)
        time.sleep(0.1)  # 短暂等待
        kinova.stop_motion()  # 立即停止
        
        print("阻抗控制和速度调整测试完成")
        
        # 速度限制控制测试
        print("\n=== 速度限制控制测试 ===")
        
        # 打印当前速度限制
        kinova.print_speed_limits()
        
        # 测试关节速度限制控制
        print("测试关节速度限制控制...")
        current_joint_limit = kinova.get_joint_speed_limits()
        if current_joint_limit is not None:
            print(f"当前关节速度限制: {current_joint_limit} 度/秒")
            
            # 增大关节速度限制
            kinova.increase_joint_speed_limits(5.0)
            
            # 减小关节速度限制
            kinova.decrease_joint_speed_limits(5.0)
        
        # 测试笛卡尔线性速度限制控制
        print("测试笛卡尔线性速度限制控制...")
        current_linear_limit = kinova.get_cartesian_linear_speed_limit()
        if current_linear_limit is not None:
            print(f"当前笛卡尔线性速度限制: {current_linear_limit} 米/秒")
            
            # 增大线性速度限制
            kinova.increase_cartesian_linear_speed_limit(0.05)
            
            # 减小线性速度限制
            kinova.decrease_cartesian_linear_speed_limit(0.05)
        
        # 测试笛卡尔角速度限制控制
        print("测试笛卡尔角速度限制控制...")
        current_angular_limit = kinova.get_cartesian_angular_speed_limit()
        if current_angular_limit is not None:
            print(f"当前笛卡尔角速度限制: {current_angular_limit} 度/秒")
            
            # 增大角速度限制
            kinova.increase_cartesian_angular_speed_limit(5.0)
            
            # 减小角速度限制
            kinova.decrease_cartesian_angular_speed_limit(5.0)
        
        # 再次打印速度限制信息
        kinova.print_speed_limits()
        
        print("速度限制控制测试完成")
        
        # 笛卡尔轨迹控制测试
        print("\n=== 笛卡尔轨迹控制测试 ===")
        
        # 定义测试路径点（基于官方示例）
        test_waypoints = [
            (0.7, 0.0, 0.5, 0.0, 90.0, 0.0, 90.0),    # 起始点
            (0.7, 0.0, 0.33, 0.1, 90.0, 0.0, 90.0),   # 下降
            (0.7, 0.48, 0.33, 0.1, 90.0, 0.0, 90.0),  # 向右移动
            (0.61, 0.22, 0.4, 0.1, 90.0, 0.0, 90.0),  # 斜向移动
            (0.7, 0.48, 0.33, 0.1, 90.0, 0.0, 90.0),  # 返回
            (0.63, -0.22, 0.45, 0.1, 90.0, 0.0, 90.0), # 向左移动
            (0.65, 0.05, 0.33, 0.0, 90.0, 0.0, 90.0)  # 结束点
        ]
        
        print("测试标准笛卡尔轨迹...")
        success = kinova.set_ee_trajectory(test_waypoints, asynchronous=False)
        if success:
            print("标准笛卡尔轨迹执行成功")
        else:
            print("标准笛卡尔轨迹执行失败")
        
        # 等待一段时间再执行优化轨迹
        time.sleep(2.0)
        
        print("测试优化的笛卡尔轨迹...")
        success = kinova.set_ee_trajectory_with_optimization(test_waypoints, asynchronous=False)
        if success:
            print("优化的笛卡尔轨迹执行成功")
        else:
            print("优化的笛卡尔轨迹执行失败")
        
        print("笛卡尔轨迹控制测试完成")
        
    except Exception as e:
        print(f"错误: {e}")
    finally:
        # 确保连接正确关闭
        if kinova is not None:
            kinova.close()
