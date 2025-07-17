from franka_robot.tora import TORA
from scipy.spatial.transform import Rotation
from franky import Robot, Gripper, Measure
from franky import Affine, JointWaypointMotion, JointWaypoint, JointMotion, CartesianMotion, ReferenceType, JointState, CartesianWaypointMotion, CartesianWaypoint
import numpy as np

# Franka类：Franka机器人高层控制接口，封装了常用的运动、抓取、状态查询等功能
class Franka:
    def __init__(self, robot_ip, relative_dynamics_factor=0.05) -> None:
        """
        初始化Franka机器人对象。
        参数：
            robot_ip (str): 机器人IP地址
            relative_dynamics_factor (float): 运动学相对动态因子，影响速度/加速度等
        """
        self.robot = Robot(robot_ip)
        self.gripper = Gripper(robot_ip)
        self.robot.recover_from_errors()  # 确保机器人处于无错误状态

        self.tora = TORA()  # 轨迹优化与逆解规划器

        self.robot.relative_dynamics_factor = relative_dynamics_factor
        # self.robot.velocity_rel = 0.05
        self.robot.acceleration_rel = 1.0
        self.robot.jerk_rel = 1.0

        self.relative_df = relative_dynamics_factor

        self.pose_shift = [0, 0, 0]  # 末端位姿偏移
        self.start_joint_pose = []    # 预设起始关节角
        self.sup_joint_pose = []      # 预设支撑关节角

        # 设置关节阻抗和碰撞检测参数
        imp_value = 1000
        torque_threshold = 50
        force_threshold = 60
        self.robot.set_joint_impedance([imp_value]*7)
        self.robot.set_collision_behavior(
            [torque_threshold]*7,  # 各关节扭矩阈值 (Nm)
            [force_threshold]*6    # 各方向力阈值 (N)
        )

        self.robot.recover_from_errors()

    def set_soft(self):
        """设置较软的关节阻抗，适合柔顺控制。"""
        imp_value = 150
        self.robot.set_joint_impedance([imp_value]*7)

    def set_hard(self):
        """设置较硬的关节阻抗，适合刚性控制。"""
        imp_value = 700
        self.robot.set_joint_impedance([imp_value]*7)

    def speed_down(self):
        """降低运动速度。"""
        self.robot.relative_dynamics_factor = self.relative_df * 0.5

    def speed_normal(self):
        """恢复默认运动速度。"""
        self.robot.relative_dynamics_factor = self.relative_df

    def set_default_pose(self):
        """
        让机器人回到预设的默认姿态。
        先到支撑位，再到起始位。
        """
        robot_pose = self.robot.current_pose
        ee_trans = robot_pose.end_effector_pose.translation
        ee_quat = robot_pose.end_effector_pose.quaternion
        self.set_joint_pose(self.sup_joint_pose)
        self.set_joint_pose(self.start_joint_pose)

    def open_gripper(self, asynchronous=True):
        """张开夹爪。"""
        success = self.gripper.move(0.04, 0.02)

    def close_gripper(self, asynchronous=True):
        """闭合夹爪。"""
        success = self.gripper.move(0.0, 0.01)

    def set_gripper_opening(self, width, asynchronous=True):
        """
        设置夹爪开口宽度。
        参数：
            width (float): 目标开口宽度（米）
        """
        current_width = self.gripper.width
        if width > 0.01:
            width = 0.04
        else:
            width = 0.0
        if abs(current_width - width) > 0.01:
            self.gripper.move(width, 0.03)

    # def set_ee_pose(self, translation, quaternion, asynchronous=True):
    #     # 通过笛卡尔运动控制末端到达目标位姿
    #     shifted_translation = translation - self.pose_shift
    #     motion = CartesianMotion(Affine(shifted_translation, quaternion))
    #     self.robot.move(motion, asynchronous=asynchronous)

    def set_ee_pose(self, ee_trans, ee_quat, asynchronous=False):
        """
        控制末端到达指定的空间位置和姿态（四元数）。
        参数：
            ee_trans (np.ndarray): 目标末端位置
            ee_quat (np.ndarray): 目标末端四元数
            asynchronous (bool): 是否异步执行
        """
        target_ee_trans = ee_trans - self.pose_shift
        target_ee_quat = ee_quat
        current_ee_trans = self.robot.current_pose.end_effector_pose.translation
        current_ee_quat = self.robot.current_pose.end_effector_pose.quaternion
        joint_pose = self.robot.state.q
        joint_v = self.robot.current_joint_velocities
        # 轨迹规划，得到关节轨迹
        target_joint, target_velocity, _ = self.tora.plan(joint_pose, joint_v, current_ee_trans, current_ee_quat, target_ee_trans, target_ee_quat)
        if asynchronous:
            self.set_joint_pose(target_joint[5], asynchronous=asynchronous)
        else:
            self.set_joint_pose(target_joint[-1], asynchronous=asynchronous)

    def set_joint_pose(self, joint_pose, asynchronous=False):
        """
        控制机器人运动到指定关节角。
        参数：
            joint_pose (list/np.ndarray): 7维关节角
            asynchronous (bool): 是否异步执行
        """
        assert len(joint_pose) == 7
        m1 = JointMotion(joint_pose)
        self.robot.move(m1, asynchronous=asynchronous)

    def set_joint_trajectories(self, joint_trajectory, velocity_trajectory, asynchronous=False):
        """
        执行关节空间轨迹。
        参数：
            joint_trajectory (np.ndarray): 关节角轨迹序列
            velocity_trajectory (np.ndarray): 关节速度轨迹序列
            asynchronous (bool): 是否异步执行
        """
        waypoints = []
        if len(joint_trajectory) == 1:
            joint_trajectory = np.array([joint_trajectory])
        step = 3
        for i in range(0, len(joint_trajectory)-1, step):
            js = joint_trajectory[i]
            jv = joint_trajectory[i+step] - joint_trajectory[i]
            print(js, jv)
            wpoint = JointWaypoint(JointState(position=js, velocity=jv*0.5))
            if i %step == 0:
                waypoints.append(wpoint)
        wpoint = JointWaypoint(JointState(position=joint_trajectory[-1]))
        waypoints.append(wpoint)
        motion = JointWaypointMotion(waypoints)
        self.robot.move(motion, asynchronous=False)

    def get_ee_pose(self):
        """
        获取当前末端位姿（位置、四元数、欧拉角）。
        返回：
            shifted_ee_trans (np.ndarray): 末端位置
            ee_quat (np.ndarray): 末端四元数
            ee_rpy (np.ndarray): 末端欧拉角
        """
        robot_pose = self.robot.current_pose
        ee_trans = robot_pose.end_effector_pose.translation
        ee_quat = robot_pose.end_effector_pose.quaternion
        ee_rpy = Rotation.from_quat(ee_quat).as_euler('xyz')
        shifted_ee_trans = ee_trans + self.pose_shift
        return shifted_ee_trans, ee_quat, ee_rpy

    def get_joint_pose(self):
        """
        获取当前关节角度。
        返回：
            joint_pose (np.ndarray): 7维关节角
        """
        state = self.robot.state
        joint_pose = state.q
        return joint_pose

    def get_elbow_pose(self):
        """
        获取肘部位置。
        返回：
            elbow_pose (np.ndarray): 肘部空间位置
        """
        state = self.robot.state
        elbow_pose = state.elbow
        return elbow_pose
    
    def get_joint_vel(self):
        """
        获取当前关节速度。
        返回：
            np.ndarray: 7维关节速度
        """
        return self.robot.current_joint_velocities
    
    def get_gripper_width(self):
        """
        获取当前夹爪开口宽度（米）。
        返回：
            float: 夹爪宽度
        """
        return self.gripper.width
    
    def get_ee_force(self):
        """
        获取末端受力（x/y/z方向分量及合力）。
        返回：
            fx, fy, fz (float): 末端受力分量
            normal_force (float): 合力
        """
        fx, fy, fz = self.robot.state.O_F_ext_hat_K[0:3]
        normal_force = (fx ** 2 + fy ** 2 + fz ** 2) ** 0.5
        return fx, fy, fz, normal_force