from scipy.spatial.transform import Rotation
from franky import Robot, Gripper, Measure
from franky import Affine, JointWaypointMotion, JointWaypoint, CartesianMotion, ReferenceType, CartesianPoseStopMotion, CartesianWaypointMotion, CartesianWaypoint
# from franka_robot.tora import TORA
import numpy as np

class Franka:
    def __init__(self, robot_ip, relative_dynamics_factor=0.05, use_curo_ik=False) -> None:
        self.robot = Robot(robot_ip)
        self.gripper = Gripper(robot_ip)
        # self.tora = TORA()

        self.robot.relative_dynamics_factor = relative_dynamics_factor
        # self.robot.velocity_rel = 0.05
        self.robot.acceleration_rel = 0.5
        self.robot.jerk_rel = 0.5

        self.relative_df = relative_dynamics_factor

        self.pose_shift = [0, 0, 0]
        self.start_joint_pose = []
        self.sup_joint_pose = []

        imp_value = 1000
        torque_threshold = 50
        force_threshold = 60
        self.robot.set_joint_impedance([imp_value, imp_value, imp_value, imp_value, imp_value, imp_value, imp_value])
        self.robot.set_collision_behavior(
            [torque_threshold, torque_threshold, torque_threshold, torque_threshold, torque_threshold, torque_threshold, torque_threshold],  # Torque thresholds (Nm)
            [force_threshold, force_threshold, force_threshold, force_threshold, force_threshold, force_threshold]       # Force thresholds (N)
        )

        self.robot.recover_from_errors()
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

            urdf_file = "/home/xi/pwei_space/curo/test/fr3_franka_hand.urdf"
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
            list/np.ndarray: 关节角   -pi~pi
        """
        quat = quat[[3,0,1,2]]
        if guess is None:
            guess = self.get_joint_pose()
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

    def set_ee_pose_curo(self, position, quat, asynchronous=False):
        """
        使用curobo的ik求解器求解逆运动学, 并设置关节角度, 实现末端笛卡尔控制
        
        参数：
            position (list/np.ndarray): 目标位置，格式为[x, y, z]
            quat (list/np.ndarray): 目标四元数，格式为[x, y, z, w]
        """
        if not self.use_curo_ik:
            print("请先设置use_curo_ik为True")
            return

        joint_angles = self.curo_ik(position, quat)

        if joint_angles is None:
            print("替换为默认笛卡尔控制器")
            self.set_ee_pose(position, quat, asynchronous=asynchronous)
        else:
            self.set_joint_pose(joint_angles, asynchronous=asynchronous)

    def set_soft(self, imp_value=150):
        imp_value = imp_value
        self.robot.set_joint_impedance([imp_value, imp_value, imp_value, imp_value, imp_value, imp_value, imp_value])

    def set_hard(self, imp_value=1000):
        imp_value = imp_value
        self.robot.set_joint_impedance([imp_value, imp_value, imp_value, imp_value, imp_value, imp_value, imp_value])

    def speed_down(self):
        self.robot.relative_dynamics_factor = self.relative_df * 0.5

    def speed_normal(self):
        self.robot.relative_dynamics_factor = self.relative_df

    def set_default_pose(self):
        self.robot.relative_dynamics_factor = 0.1
        motion = CartesianMotion(Affine([-0.04, 0.0, 0.0]), ReferenceType.Relative)
        self.robot.move(motion)    

        robot_pose = self.robot.current_pose
        ee_trans = robot_pose.end_effector_pose.translation
        ee_quat = robot_pose.end_effector_pose.quaternion
        # motion = CartesianMotion(Affine(ee_trans+np.array([-0.05, 0.0, 0.1]), ee_quat))
        # self.robot.move(motion)
        if ee_trans[0] > 0.5:
            self.set_joint_pose(self.sup_joint_pose)

        self.set_joint_pose(self.start_joint_pose)
        self.robot.relative_dynamics_factor = self.relative_df

    def open_gripper(self, asynchronous=True):
        # if asynchronous:
        #     self.gripper.move_async(0.04, 0.02)
        # else:
        #     self.gripper.open(0.04)
        success = self.gripper.move(0.04, 0.02)

    def close_gripper(self, asynchronous=True):
        # if asynchronous:
        #     self.gripper.move_async(0.0, 0.03)
        # else:
        #     self.gripper.move(0.0, 0.03)
        success = self.gripper.move(0.002, 0.03)
        # self.gripper.grasp(0.0, 0.05, 20, epsilon_outer=1.0)

    def set_gripper_opening(self, width, asynchronous=True):
        current_width = self.gripper.width
        # if asynchronous:
        #     self.gripper.move_async(width, 0.02)
        # else:
        if width > 0.01:
            width = 0.04
        else:
            width = 0.0

        if abs(current_width - width) > 0.01:
            self.gripper.move(width, 0.03)
        # success = self.gripper.move(0.0, 0.02)

    def set_ee_pose(self, translation, quaternion, asynchronous=True):
        # print('set ee')
        shifted_translation = translation - self.pose_shift
        motion = CartesianMotion(Affine(shifted_translation, quaternion))
        self.robot.move(motion, asynchronous=asynchronous)

    def set_ee_pose_relative(self, translation, asynchronous=False):
        # shifted_translation = translation - self.pose_shift
        motion = CartesianMotion(Affine(translation), ReferenceType.Relative)
        self.robot.move(motion, asynchronous=asynchronous)

    def set_sora_pose(self, ee_trans, ee_quat, asynchronous=False):
        target_ee_trans = ee_trans - self.pose_shift
        target_ee_quat = ee_quat
        current_ee_trans = self.robot.current_pose.end_effector_pose.translation
        current_ee_quat = self.robot.current_pose.end_effector_pose.quaternion
        joint_pose = self.robot.state.q
        joint_v = self.robot.current_joint_velocities

        target_joint, _, _ = self.tora.plan(joint_pose, joint_v, current_ee_trans, current_ee_quat, target_ee_trans, target_ee_quat)
        # return target_joint
        self.set_joint_pose(target_joint[-1], asynchronous=asynchronous)

    def set_joint_pose(self, joint_pose, asynchronous=False):
        assert len(joint_pose) == 7
        m1 = JointWaypointMotion([JointWaypoint(joint_pose)])
        self.robot.move(m1, asynchronous=asynchronous)

    def set_joint_trajectories(self, joint_trajectory, asynchronous=False):
        waypoints = []
        if len(joint_trajectory) == 1:
            joint_trajectory = np.array([joint_trajectory])

        print(joint_trajectory)
        for i in range(len(joint_trajectory)):
            js = joint_trajectory[i]
            wpoint = JointWaypoint(js)
            # wpoint = JointWaypoint(JointState(position=js, velocity=jv))
            waypoints.append(wpoint)
        motion = JointWaypointMotion(waypoints)
        # m1 = JointWaypointMotion([JointWaypoint(joint_pose)])
        self.robot.move(motion, asynchronous=asynchronous)

    def get_ee_pose(self):
        robot_pose = self.robot.current_pose
        ee_trans = robot_pose.end_effector_pose.translation
        ee_quat = robot_pose.end_effector_pose.quaternion
        ee_rpy = Rotation.from_quat(ee_quat).as_euler('xyz')

        shifted_ee_trans = ee_trans + self.pose_shift
        return shifted_ee_trans, ee_quat, ee_rpy

    def get_joint_pose(self):
        state = self.robot.state
        joint_pose = state.q
        return joint_pose

    def get_elbow_pose(self):
        state = self.robot.state
        elbow_pose = state.elbow
        return elbow_pose
    
    def get_joint_vel(self):
        return self.robot.current_joint_velocities
    
    def get_gripper_width(self):
        return self.gripper.width
    
    def get_ee_force(self):
        fx, fy, fz = self.robot.state.O_F_ext_hat_K[0:3]
        normal_force = (fx ** 2 + fy ** 2 + fz ** 2) ** 0.5
        # print(self.robot.state.O_F_ext_hat_K, normal_force)
        # normal_force = (fx ** 2 + fy ** 2 + fz ** 2) ** 0.5
        return fx, fy, fz, normal_force