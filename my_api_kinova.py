#! /usr/bin/env python3

"""
cartesian_move_api.py
---------------------
Kinova Gen3/G3L机械臂常用运动API集合。
封装了笛卡尔空间、关节空间、轨迹、正逆运动学等常用控制与查询接口，便于二次开发与集成。
"""

import sys
import os
import threading
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.messages import Base_pb2

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

def cartesian_move(base, base_cyclic, dx=0, dy=0, dz=0, dtheta_x=0, dtheta_y=0, dtheta_z=0, timeout=TIMEOUT_DURATION):
    """
    末端笛卡尔空间相对运动。
    以当前末端位姿为基准，增量移动到新位置。
    
    参数：
        base (BaseClient): 机械臂Base服务客户端。
        base_cyclic (BaseCyclicClient): 机械臂BaseCyclic服务客户端。
        dx, dy, dz (float): 末端在x/y/z方向的相对位移（单位：米）。
        dtheta_x, dtheta_y, dtheta_z (float): 末端绕x/y/z轴的相对旋转（单位：度）。
        timeout (float): 超时时间（秒），默认20。
    返回：
        bool: True-动作完成，False-超时/中止。
    注意：
        - 增量过大可能导致动作失败或超出机械臂工作空间。
        - 推荐每次移动量不超过机械臂最大步长。
    示例：
        cartesian_move(base, base_cyclic, dx=0.1, dy=0, dz=0)
    """
    feedback = base_cyclic.RefreshFeedback()
    action = Base_pb2.Action()
    action.name = "API Cartesian Move"
    action.application_data = ""
    cartesian_pose = action.reach_pose.target_pose
    cartesian_pose.x = feedback.base.tool_pose_x + dx
    cartesian_pose.y = feedback.base.tool_pose_y + dy
    cartesian_pose.z = feedback.base.tool_pose_z + dz
    cartesian_pose.theta_x = feedback.base.tool_pose_theta_x + dtheta_x
    cartesian_pose.theta_y = feedback.base.tool_pose_theta_y + dtheta_y
    cartesian_pose.theta_z = feedback.base.tool_pose_theta_z + dtheta_z
    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )
    print(f"[cartesian_move] 增量移动: dx={dx}, dy={dy}, dz={dz}, dtheta_x={dtheta_x}, dtheta_y={dtheta_y}, dtheta_z={dtheta_z}")
    base.ExecuteAction(action)
    finished = e.wait(timeout)
    base.Unsubscribe(notification_handle)
    if finished:
        print("[cartesian_move] 动作完成。")
    else:
        print("[cartesian_move] 超时或中止。")
    return finished

def joint_move(base, joint_angles, timeout=TIMEOUT_DURATION):
    """
    关节空间运动。
    控制机械臂各关节移动到指定角度。
    
    参数：
        base (BaseClient): 机械臂Base服务客户端。
        joint_angles (list[float]): 长度为7的关节角度列表（单位：度，顺序为关节1~7）。
        timeout (float): 超时时间（秒），默认20。
    返回：
        bool: True-动作完成，False-超时/中止。
    注意：
        - 角度超出机械臂物理限制会导致动作失败。
        - 关节顺序需与机械臂实际顺序一致。
    示例：
        joint_move(base, [0, 0, 0, 0, 0, 0, 0])
    """
    if not isinstance(joint_angles, (list, tuple)) or len(joint_angles) != 7:
        raise ValueError("joint_angles 必须为长度为7的列表或元组")
    action = Base_pb2.Action()
    action.name = "API Joint Move"
    action.application_data = ""
    for joint_id, angle in enumerate(joint_angles):
        joint_angle = action.reach_joint_angles.joint_angles.joint_angles.add()
        joint_angle.joint_identifier = joint_id
        joint_angle.value = angle
    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )
    print(f"[joint_move] 目标关节角: {joint_angles}")
    base.ExecuteAction(action)
    finished = e.wait(timeout)
    base.Unsubscribe(notification_handle)
    if finished:
        print("[joint_move] 动作完成。")
    else:
        print("[joint_move] 超时或中止。")
    return finished

def cartesian_sequnence_move(base, poses, blending_radius=0.0, timeout=30):
    """
    笛卡尔空间轨迹运动。
    控制机械臂末端依次经过给定的空间点，自动插值平滑运动。
    
    参数：
        base (BaseClient): 机械臂Base服务客户端。
        poses (list[tuple[float]]): 路点序列，每个元素为(x, y, z, theta_x, theta_y, theta_z)。
        blending_radius (float): 路点平滑半径（米），默认0.0。
        timeout (float): 超时时间（秒），默认30。
    返回：
        bool: True-轨迹完成，False-超时/中止。
    注意：
        - 路点数量建议2~20，过多可能导致超时。
        - blending_radius>0可实现轨迹平滑过渡。
        - 路点超出工作空间或姿态异常会导致轨迹验证失败。
    示例：
        poses = [(0.5,0,0.3,0,0,0), (0.6,0,0.3,0,0,0)]
        cartesian_sequnence_move(base, poses, blending_radius=0.05)
    """
    from kortex_api.autogen.messages import Base_pb2
    waypoints = Base_pb2.WaypointList()
    waypoints.duration = 0.0
    waypoints.use_optimal_blending = False
    for idx, pose in enumerate(poses):
        waypoint = waypoints.waypoints.add()
        waypoint.name = f"waypoint_{idx}"
        cartesian_wp = waypoint.cartesian_waypoint
        cartesian_wp.pose.x = pose[0]
        cartesian_wp.pose.y = pose[1]
        cartesian_wp.pose.z = pose[2]
        cartesian_wp.pose.theta_x = pose[3]
        cartesian_wp.pose.theta_y = pose[4]
        cartesian_wp.pose.theta_z = pose[5]
        cartesian_wp.blending_radius = blending_radius
        cartesian_wp.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_BASE
    result = base.ValidateWaypointList(waypoints)
    if len(result.trajectory_error_report.trajectory_error_elements) != 0:
        print("[cartesian_sequnence_move] 轨迹验证失败，存在无效路点。")
        result.trajectory_error_report.PrintDebugString()
        return False
    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )
    print(f"[cartesian_sequnence_move] 路点数: {len(poses)}，开始执行轨迹。")
    base.ExecuteWaypointTrajectory(waypoints)
    finished = e.wait(timeout)
    base.Unsubscribe(notification_handle)
    if finished:
        print("[cartesian_sequnence_move] 轨迹完成。")
    else:
        print("[cartesian_sequnence_move] 超时或中止。")
    return finished

def cartesian_set(base, base_cyclic, x, y, z, theta_x, theta_y, theta_z, timeout=TIMEOUT_DURATION):
    """
    末端笛卡尔空间绝对运动。
    控制机械臂末端移动到指定的绝对位姿。
    
    参数：
        base (BaseClient): 机械臂Base服务客户端。
        base_cyclic (BaseCyclicClient): 机械臂BaseCyclic服务客户端。
        x, y, z (float): 目标末端位置（米）。
        theta_x, theta_y, theta_z (float): 目标末端姿态（度）。
        timeout (float): 超时时间（秒），默认20。
    返回：
        bool: True-动作完成，False-超时/中止。
    注意：
        - 目标位姿应在机械臂可达空间内。
        - 姿态角度单位为度，范围通常为-180~180。
    示例：
        cartesian_set(base, base_cyclic, 0.5, 0, 0.3, 0, 0, 0)
    """
    from kortex_api.autogen.messages import Base_pb2
    action = Base_pb2.Action()
    action.name = "API Cartesian Set"
    action.application_data = ""
    cartesian_pose = action.reach_pose.target_pose
    cartesian_pose.x = x
    cartesian_pose.y = y
    cartesian_pose.z = z
    cartesian_pose.theta_x = theta_x
    cartesian_pose.theta_y = theta_y
    cartesian_pose.theta_z = theta_z
    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )
    print(f"[cartesian_set] 目标位姿: x={x}, y={y}, z={z}, theta_x={theta_x}, theta_y={theta_y}, theta_z={theta_z}")
    base.ExecuteAction(action)
    finished = e.wait(timeout)
    base.Unsubscribe(notification_handle)
    if finished:
        print("[cartesian_set] 已到达目标位姿。")
    else:
        print("[cartesian_set] 超时或中止。")
    return finished

def get_joint_torque(base_cyclic):
    """
    查询当前七个关节的力矩（扭矩）。
    
    参数：
        base_cyclic (BaseCyclicClient): 机械臂BaseCyclic服务客户端。
    返回：
        list[float]: 七个关节的力矩列表（N·m）。
    注意：
        - 返回顺序与机械臂关节顺序一致。
        - 若实际关节数不足7，自动补零。
    示例：
        torques = get_joint_torque(base_cyclic)
    """
    feedback = base_cyclic.RefreshFeedback()
    torques = [actuator.torque for actuator in feedback.actuators]
    while len(torques) < 7:
        torques.append(0.0)
    return torques[:7]

def get_joint_angles(base_cyclic, use_radian=False):
    """
    查询当前七个关节的关节角。
    
    参数：
        base_cyclic (BaseCyclicClient): 机械臂BaseCyclic服务客户端。
        use_radian (bool): True-返回弧度，False-返回角度（默认）。
    返回：
        list[float]: 七个关节角度列表（度或弧度）。
    注意：
        - 返回顺序与机械臂关节顺序一致。
        - 若实际关节数不足7，自动补零。
    示例：
        angles = get_joint_angles(base_cyclic, use_radian=True)
    """
    import math
    feedback = base_cyclic.RefreshFeedback()
    angles = []
    for actuator in feedback.actuators:
        angle = actuator.position
        if use_radian:
            angle = math.radians(angle)
        angles.append(angle)
    while len(angles) < 7:
        angles.append(0.0)
    return angles[:7]

def compute_fk(base, joint_angles):
    """
    正运动学：根据关节角计算末端位姿。
    
    参数：
        base (BaseClient): 机械臂Base服务客户端。
        joint_angles (list[float]): 长度为7的关节角度列表（度）。
    返回：
        tuple[float]: (x, y, z, theta_x, theta_y, theta_z)
    注意：
        - 输入角度顺序需与机械臂实际顺序一致。
        - 超出物理极限的角度可能导致结果无效。
    示例：
        pose = compute_fk(base, [0,0,0,0,0,0,0])
    """
    from kortex_api.autogen.messages import Base_pb2
    if not isinstance(joint_angles, (list, tuple)) or len(joint_angles) != 7:
        raise ValueError("joint_angles 必须为长度为7的列表或元组")
    joint_angles_msg = Base_pb2.JointAngles()
    for idx, angle in enumerate(joint_angles):
        joint_angle = joint_angles_msg.joint_angles.add()
        joint_angle.joint_identifier = idx
        joint_angle.value = angle
    pose = base.ComputeForwardKinematics(joint_angles_msg)
    return (pose.x, pose.y, pose.z, pose.theta_x, pose.theta_y, pose.theta_z)

def compute_ik(base, pose, guess=None):
    """
    逆运动学：根据末端位姿计算关节角。
    
    参数：
        base (BaseClient): 机械臂Base服务客户端。
        pose (tuple[float]): 末端位姿 (x, y, z, theta_x, theta_y, theta_z)。
        guess (list[float]|None): 可选，长度为7的关节角初始猜测（度），有助于收敛。
    返回：
        list[float]: 七个关节角度列表（度）。
    注意：
        - 逆解可能不唯一，初始猜测可影响收敛结果。
        - 目标位姿超出可达空间时会抛出异常。
    示例：
        angles = compute_ik(base, (0.5,0,0.3,0,0,0), guess=[0,0,0,0,0,0,0])
    """
    from kortex_api.autogen.messages import Base_pb2
    if not isinstance(pose, (list, tuple)) or len(pose) != 6:
        raise ValueError("pose 必须为长度为6的元组或列表")
    ik_data = Base_pb2.IKData()
    ik_data.cartesian_pose.x = pose[0]
    ik_data.cartesian_pose.y = pose[1]
    ik_data.cartesian_pose.z = pose[2]
    ik_data.cartesian_pose.theta_x = pose[3]
    ik_data.cartesian_pose.theta_y = pose[4]
    ik_data.cartesian_pose.theta_z = pose[5]
    if guess is not None:
        if not isinstance(guess, (list, tuple)) or len(guess) != 7:
            raise ValueError("guess 必须为长度为7的列表或元组")
        for angle in guess:
            jAngle = ik_data.guess.joint_angles.add()
            jAngle.value = angle
    joint_angles_msg = base.ComputeInverseKinematics(ik_data)
    return [ja.value for ja in joint_angles_msg.joint_angles]

# =====================
# 示例主程序
# =====================

def main():
    """
    API功能测试主程序。
    连接机械臂后，依次演示各API典型用法。
    """
    # sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    import utilities
    args = utilities.parseConnectionArguments()
    with utilities.DeviceConnection.createTcpConnection(args) as router:
        base = BaseClient(router)
        base_cyclic = BaseCyclicClient(router)
        # =======================================
        # 示例1：末端笛卡尔空间增量运动
        # cartesian_move(base, base_cyclic, dx=0, dy=+0.1, dz=+0.1, dtheta_x=0, dtheta_y=0, dtheta_z=0)
        # =======================================
        # 示例2：关节空间运动
        # joint_move(base, [0, 0, 0, 0, 0, 0, 0])
        # =======================================
        # 示例3：查询关节力矩
        # torques = get_joint_torque(base_cyclic)
        # print(f"[get_joint_torque] 当前七个关节的力矩值: {torques}")
        # =======================================
        # 示例4：查询关节角（角度制）
        # joint_angles_deg = get_joint_angles(base_cyclic, use_radian=False)
        # print(f"[get_joint_angles] 当前七个关节的角度（度）: {joint_angles_deg}")
        # =======================================
        # 示例5：查询关节角（弧度制）
        # joint_angles_rad = get_joint_angles(base_cyclic, use_radian=True)
        # print(f"[get_joint_angles] 当前七个关节的角度（弧度）: {joint_angles_rad}")
        # =======================================
        # 示例6：末端绝对位姿运动
        # feedback = base_cyclic.RefreshFeedback()
        # target_x = feedback.base.tool_pose_x + 0.05
        # target_y = feedback.base.tool_pose_y
        # target_z = feedback.base.tool_pose_z
        # target_theta_x = feedback.base.tool_pose_theta_x
        # target_theta_y = feedback.base.tool_pose_theta_y
        # target_theta_z = feedback.base.tool_pose_theta_z
        # cartesian_set(base, base_cyclic, target_x, target_y, target_z, target_theta_x, target_theta_y, target_theta_z)
        # =======================================
        # 示例7：笛卡尔空间轨迹运动
        feedback = base_cyclic.RefreshFeedback()
        start_pose = (
            feedback.base.tool_pose_x,
            feedback.base.tool_pose_y,
            feedback.base.tool_pose_z,
            feedback.base.tool_pose_theta_x,
            feedback.base.tool_pose_theta_y,
            feedback.base.tool_pose_theta_z
        )
        target_pose = (
            feedback.base.tool_pose_x + 0.1,
            feedback.base.tool_pose_y,
            feedback.base.tool_pose_z,
            feedback.base.tool_pose_theta_x,
            feedback.base.tool_pose_theta_y,
            feedback.base.tool_pose_theta_z
        )
        poses = [start_pose, target_pose]
        cartesian_sequnence_move(base, poses, blending_radius=0.0, timeout=30)
        # =======================================
        # 示例8：正运动学
        joint_angles_now = get_joint_angles(base_cyclic, use_radian=False)
        fk_pose = compute_fk(base, joint_angles_now)
        print(f"[compute_fk] 当前关节角的末端位姿: {fk_pose}")
        # =======================================
        # 示例9：逆运动学
        pose_now = (
            feedback.base.tool_pose_x,
            feedback.base.tool_pose_y,
            feedback.base.tool_pose_z,
            feedback.base.tool_pose_theta_x,
            feedback.base.tool_pose_theta_y,
            feedback.base.tool_pose_theta_z
        )
        ik_angles = compute_ik(base, pose_now, guess=joint_angles_now)
        print(f"[compute_ik] 当前末端位姿的逆解关节角: {ik_angles}")

if __name__ == "__main__":
    main() 