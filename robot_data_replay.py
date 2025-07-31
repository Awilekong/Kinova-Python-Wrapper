#!/usr/bin/env python3
"""
franka_data_replay.py
---------------------
Franka机械臂数据回放脚本

功能：
1. 回放录制的轨迹数据到Franka机械臂
2. 可视化录制的RGB/深度图像序列
3. 支持轨迹数据的浏览和选择
4. 支持键盘交互控制回放过程

主要模式：
- rgb/depth模式：可视化图像序列
- trajectory模式：机械臂轨迹回放
"""

# from juliacall import Main as jl, convert as jlconvert
# jl.seval("using MeshCat")
# jl.seval("using Rotations")
# jl.seval("using TORA")

import numpy as np
import rospy
import tf
import tf2_ros
import geometry_msgs.msg
import signal
import os
import argparse
import cv2
import h5py
# from franka_robot.franka_dual_arm import FrankaLeft, FrankaRight
from kinova_basic import Kinova
import math
from tf.transformations import quaternion_from_euler, quaternion_multiply

from scipy.spatial.transform import Rotation


# rospy.init_node("your_node_name")
# broadcaster = tf.TransformBroadcaster()
# broadcaster.sendTransform(
#     (1.01-0.025, 0.3875, 0.042),  # 平移
#     (0, 0, 1, 0),  # 四元数
#     rospy.Time.now(),
#     "kinova_base",
#     "table"
# )

def signal_handler(sig, frame):
    """Ctrl+C信号处理函数"""
    print('You pressed Ctrl+C!')
    # rospy.signal_shutdown("You pressed Ctrl+C!")
    exit(0)

def get_trajectory(path, idx=0):
    """
    获取轨迹数据文件
    
    参数：
        path: 数据路径
        idx: 文件索引（0表示最新文件）
    返回：
        data: HDF5文件对象
    """
    # 获取最后一个录制的轨迹
    f_list = os.listdir(path)
    f_num = len(f_list)
    f_idx = min(idx, f_num-1)
    print('selected file id', f_idx, f_num-1)
    file_path = path + '/' + str(f_idx) + '.h5'

    data = h5py.File(file_path, 'r')
    return data

def get_data_list(traj_path, mode, idx):
    """
    获取数据列表
    
    参数：
        traj_path: 轨迹路径
        mode: 模式（'rgb'或'depth'）
        idx: 文件索引
    返回：
        trans_list: 位置列表
        quat_list: 姿态列表
        gripper_list: 夹爪状态列表
        img_list: 图像列表
    """
    data = get_trajectory(traj_path, idx=idx)
    for keys in data:
        print(keys, len(data[keys]))

    trans_list, quat_list, gripper_list = np.array(data['translation']), np.array(data['rotation']), np.array(data['gripper_w'])
    if mode == 'rgb':
        img_list = np.array(data['rgb'])
    elif mode == 'depth':
        img_list = np.array(data['depth'])

    return trans_list, quat_list, gripper_list, img_list

def create_transform_matrix(axis_mapping, translation):
    """
    基于轴映射创建旋转矩阵 R_{A->B} 和平移向量 t_B（B 中表示）。

    参数：
        axis_mapping: {'kinova_x': 'table_-x', …}，表示 B 轴相对于 A 轴的方向。
        translation: np.array，B 原点相对于 A 原点的偏移，已在 A 坐标系下表示。

    返回：
        t_B: np.array，平移向量，在 B 坐标系下表示（带上了负号）。
        R: np.array(3,3)，旋转矩阵 R_{A->B}。
    """
    # 构造 R 使得：对于任意以 B 轴表示的向量 v_B，R @ v_B = v_A
    R = np.zeros((3,3))
    for kinova_axis, table_axis in axis_mapping.items():
        idx = {'kinova_x':0, 'kinova_y':1, 'kinova_z':2}[kinova_axis]
        vec = {
            'table_x':  [1,0,0],
            'table_y':  [0,1,0],
            'table_z':  [0,0,1],
            'table_-x': [-1,0,0],
            'table_-y': [0,-1,0],
            'table_-z': [0,0,-1],
        }[table_axis]
        R[:, idx] = vec

    # 验证正交性
    if not np.allclose(R @ R.T, np.eye(3), atol=1e-6):
        raise ValueError("R 不是正交矩阵，请检查 axis_mapping")

    # 计算 B 下的平移： t_B = -R^T * t_A
    t_B = -R.T @ translation
    return t_B, R

def transform_pose_table_to_kinova(pose, translation, axis_mapping):
    """
    把一个位姿 (位置, 四元数) 从 table(A) 坐标系 转到 kinova(B) 坐标系。
    """
    trans_A, quat_A = pose
    t_B, R = create_transform_matrix(axis_mapping, translation)

    # 位置部分： p_B = R^T p_A + t_B  （t_B 已带负号）
    trans_B = R.T @ trans_A + t_B
    from scipy.spatial.transform import Rotation
    # 姿态部分： R_B = R^T * R_A
    R_orig = Rotation.from_quat(quat_A)
    R_new  = Rotation.from_matrix(R.T) * R_orig
    quat_B = R_new.as_quat()

    return trans_B, quat_B

def transform_pose_simple(p_A, q_A, t_A_to_B):
    """
    把位姿从 table (A) 转到 kinova (B)，
    映射：x_B = -x_A，y_B = -y_A，z_B = z_A。

    参数：
        p_A: np.array([x,y,z])，点在 A 系中的坐标
        q_A: np.array([x,y,z,w])，四元数（SciPy xyzw 顺序）
        t_A_to_B: np.array([tx,ty,tz])，B 原点在 A 系的坐标

    返回：
        p_B, q_B：在 B 系的位姿
    """
    # 1. 平移：先做 (p_A - t)，再做符号翻转
    dp = p_A - t_A_to_B
    p_B = np.array([-dp[0], -dp[1],  dp[2]])
    from scipy.spatial.transform import Rotation as R
    # 2. 姿态：R = diag([-1,-1,1]) 就是绕 Z 轴转 180°
    #    Quaternion 表示为 q_rot = [0,0,1,0] （xyzw）
    q_rot = np.array([0.0, 0.0, 1.0, 0.0])
    R_rot = R.from_quat(q_rot)

    R_orig = R.from_quat(q_A)
    R_new  = R_rot * R_orig   # 先做原始，再做 180°绕 Z
    q_B    = R_new.as_quat()

    return p_B, q_B

def transform_A_to_B_tf(p_A, q_A):
    """
    使用ROS TF进行table到kinova_base的坐标变换。
    p_A: np.array([x, y, z])，table系下位置
    q_A: np.array([x, y, z, w])，table系下四元数（xyzw）
    返回: p_B, q_B，kinova_base系下位置和四元数
    """
    listener = tf.TransformListener()
    # 等待变换可用
    try:
        listener.waitForTransform("kinova_base", "table", rospy.Time(), rospy.Duration(4.0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logwarn("Transform not available")
        return None, None
    # 构造PointStamped和QuaternionStamped
    # p_msg = geometry_msgs.msg.PointStamped()
    # p_msg.header.frame_id = "table"
    # p_msg.header.stamp = rospy.Time(0)
    # p_msg.point.x, p_msg.point.y, p_msg.point.z = p_A.tolist()
    # q_msg = geometry_msgs.msg.QuaternionStamped()
    # q_msg.header.frame_id = "table"
    # q_msg.header.stamp = rospy.Time(0)
    # q_msg.quaternion.x, q_msg.quaternion.y, q_msg.quaternion.z, q_msg.quaternion.w = q_A.tolist()

    # Your original point in franka_base frame
    p_trans = p_A  # Replace with your actual coordinates
    p_quat = q_A  # Replace with your actual quaternion

    pose_stamped = geometry_msgs.msg.PoseStamped()
    pose_stamped.header.frame_id = "table"
    pose_stamped.header.stamp = rospy.Time(0)
    pose_stamped.pose.position.x = p_trans[0]
    pose_stamped.pose.position.y = p_trans[1]
    pose_stamped.pose.position.z = p_trans[2]
    pose_stamped.pose.orientation.x = p_quat[0]
    pose_stamped.pose.orientation.y = p_quat[1]
    pose_stamped.pose.orientation.z = p_quat[2]
    pose_stamped.pose.orientation.w = p_quat[3]

    try:
        transformed_pose = listener.transformPose("kinova_base", pose_stamped)
        p_B = np.array([transformed_pose.pose.position.x, transformed_pose.pose.position.y, transformed_pose.pose.position.z])
        q_B = np.array([transformed_pose.pose.orientation.x, transformed_pose.pose.orientation.y, transformed_pose.pose.orientation.z, transformed_pose.pose.orientation.w])
        # print('p_B', p_B)
        # print('q_B', q_B)
        return p_B, q_B
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logwarn("Transform failed")
        return None, None

def batch_transform_A_to_B_tf(p_A_list, q_A_list):
    """
    批量使用ROS TF进行table到kinova_base的坐标变换。
    p_A_list: list of np.array([x, y, z])，table系下位置列表
    q_A_list: list of np.array([x, y, z, w])，table系下四元数列表
    返回: p_B_list, q_B_list，kinova_base系下位置和四元数列表
    """
    listener = tf.TransformListener()
    # 等待变换可用
    try:
        listener.waitForTransform("kinova_base", "table", rospy.Time(), rospy.Duration(4.0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logwarn("Transform not available")
        return [], []

    p_B_list = []
    q_B_list = []
    for p_A, q_A in zip(p_A_list, q_A_list):
        pose_stamped = geometry_msgs.msg.PoseStamped()
        pose_stamped.header.frame_id = "table"
        pose_stamped.header.stamp = rospy.Time(0)
        pose_stamped.pose.position.x = p_A[0]
        pose_stamped.pose.position.y = p_A[1]
        pose_stamped.pose.position.z = p_A[2]
        pose_stamped.pose.orientation.x = q_A[0]
        pose_stamped.pose.orientation.y = q_A[1]
        pose_stamped.pose.orientation.z = q_A[2]
        pose_stamped.pose.orientation.w = q_A[3]

        try:
            transformed_pose = listener.transformPose("kinova_base", pose_stamped)
            p_B = np.array([transformed_pose.pose.position.x, transformed_pose.pose.position.y, transformed_pose.pose.position.z])
            q_B = np.array([transformed_pose.pose.orientation.x, transformed_pose.pose.orientation.y, transformed_pose.pose.orientation.z, transformed_pose.pose.orientation.w])
            p_B_list.append(p_B)
            q_B_list.append(q_B)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Transform failed for one pose")
            p_B_list.append(None)
            q_B_list.append(None)
    return p_B_list, q_B_list

def publish_static_transform():
    """发布静态TF变换：从table到kinova_base"""
    static_broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transformStamped = geometry_msgs.msg.TransformStamped()

    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "table"
    static_transformStamped.child_frame_id = "kinova_base"

    static_transformStamped.transform.translation.x = 1.01+0.09
    static_transformStamped.transform.translation.y = 0.39
    static_transformStamped.transform.translation.z = 0.042

    static_transformStamped.transform.rotation.x = 0.0
    static_transformStamped.transform.rotation.y = 0.0
    static_transformStamped.transform.rotation.z = 1.0
    static_transformStamped.transform.rotation.w = 0.0

    static_broadcaster.sendTransform(static_transformStamped)
    return static_broadcaster  # 返回广播器对象以保持其生命周期

if __name__ == '__main__':
    # 系统初始化
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node("robot_data_replay")
    rate = rospy.Rate(10)  # 10Hz循环频率

    # 发布静态TF变换
    # while True:
    static_broadcaster = publish_static_transform()
    rospy.sleep(1.0)  # 等待TF变换建立

    # 命令行参数解析
    parser = argparse.ArgumentParser()
    parser.add_argument('--base_path', default='./paper_hdf5_v4/human', type=str)  # 数据基础路径
    parser.add_argument('--arm', default='kinova', type=str)        # 机械臂：left, right        
    parser.add_argument('--cam', default='cam_up', type=str)      # 相机：up, down（初始夹爪位置）
    parser.add_argument('--zip', default='zip_top', type=str)     # 拉链：top, bottom（拉链位置）
    parser.add_argument('--item', default='small_box', type=str)  # 物品类型
    parser.add_argument('--data_mode', default='grasp', type=str) # 数据模式：grasp, open, grasp_noise, open_noise
    parser.add_argument('--idx', default=5, type=int)             # 文件索引
    parser.add_argument('--mode', default='trajectory', type=str)        # 模式：rgb, depth, trajectory
    args = parser.parse_args()

    # 构建轨迹路径
    # traj_path = os.path.join(args.base_path, args.data_mode, args.item, args.zip, args.cam)
    traj_path = './data'
    f_list = os.listdir(traj_path)
    f_num = len(f_list)    

    if args.mode == 'rgb' or args.mode == 'depth':
        """
        RGB/深度图像可视化模式
        支持键盘交互：
        - 左右箭头：浏览同一轨迹的不同帧
        - n键：下一个轨迹文件
        - p键：上一个轨迹文件
        - ESC键：退出
        """
        base_idx = args.idx
        trans_list, quat_list, gripper_list, img_list = get_data_list(traj_path, mode=args.mode, idx=base_idx)
        len_list = len(img_list)
        idx = 0
        print('last trans', trans_list[-1])

        while True:
            # 获取当前帧
            frame = img_list[idx]
            print(idx, frame.shape, gripper_list[idx])
            
            # 图像裁剪：只显示中间部分（去除边缘）
            col_shift = 200
            frame = frame[:300, 320-col_shift:320+col_shift, :]  # 裁剪顶部300行，中间400列

            # frame = cv2.resize(frame, (224, 224))  # 可选：调整图像大小
            cv2.imshow('Align Example', frame)
            
            # 键盘交互处理
            key = cv2.waitKey(0)
            if key == 110:  # 'n'键：下一个轨迹
                base_idx = min(f_num-1, base_idx + 1)
                print('\n -- idx', base_idx)
                trans_list, quat_list, gripper_list, img_list = get_data_list(traj_path, mode=args.mode, idx=base_idx)
                len_list = len(img_list)
                idx = 0
            elif key == 112:  # 'p'键：上一个轨迹
                base_idx = max(0, base_idx - 1)
                print('\n -- idx', base_idx)
                trans_list, quat_list, gripper_list, img_list = get_data_list(traj_path, mode=args.mode, idx=base_idx)
                len_list = len(img_list)
                idx = 0
            elif key == 81:   # 左箭头：上一帧
                idx = max(0, idx-1)
            elif key == 83:   # 右箭头：下一帧
                idx = min(len_list-1, idx+1)
            elif key == 27:   # ESC键：退出
                break
        print(img_list.shape)

    elif args.mode == 'trajectory':
        """
        轨迹回放模式
        将录制的轨迹数据回放到Franka机械臂
        """
        # 初始化机械臂
        # if 'left' in args.arm:
        #     arm = FrankaLeft() 
        # elif 'right' in args.arm:
        #     arm = FrankaRight()
        # else:
        #     arm = Kinova()
        #     # Kinova机械臂：更新工具配置
        #     from kinova_tool_configuration import KinovaToolConfiguration
        #     tool_config = KinovaToolConfiguration(arm)
        #     print("正在更新Kinova工具配置...")
        #     if tool_config.set_default_gripper_config():
        #         print("✓ Kinova工具配置更新成功")
        #     else:
        #         print("✗ Kinova工具配置更新失败，使用默认配置")
        arm = Kinova()
        # Kinova机械臂：更新工具配置
        from kinova_tool_configuration import KinovaToolConfiguration
        tool_config = KinovaToolConfiguration(arm)
        print("正在更新Kinova工具配置...")
        if tool_config.set_default_gripper_config():
            print("✓ Kinova工具配置更新成功")
        else:
            print("✗ Kinova工具配置更新失败，使用默认配置")

        # 机械臂初始化
        arm.open_gripper()  # 张开夹爪
        
        # 预设的关节角度位置（用于不同相机视角）
        joint_up = np.array([-0.97788733, -1.04903993,  1.31520369, -1.58949637, -0.26875838,  1.36971498, 2.23423306])
        joint_down = np.array([-1.12243027, -1.2869527, 1.72586445, -2.25379698,  0.18903419, 2.15440121, 2.43160574])
        
        # 移动到预设位置
        # arm.set_joint_pose(joint_up, asynchronous=False)
        
        # 等待用户确认开始回放
        input("Press Enter to start the trajectory playback...")
        shift_franka = np.array([-1.0+0.025, 0.0, 0.015])
        shift_kinova = np.array([0.01, 0.132, 0.0435])
        shift_kinova_test = np.array([1.01-0.025, 0.3875, 0.042])
        axis_mapping = {
                'kinova_x': 'table_-x',    # Kinova的X轴 = table的负X轴
                'kinova_y': 'table_-y',    # Kinova的Y轴 = table的负Y轴
                'kinova_z': 'table_z'      # Kinova的Z轴 = table的Z轴
            }
        # 获取轨迹数据
        data = get_trajectory(traj_path, idx=args.idx)
        trans_list, quat_list, gripper_list = np.array(data['translation']), np.array(data['rotation']), np.array(data['gripper_w'])
        # 90° 转成弧度
        ry90 = math.radians(90)
        # axes='sxyz' 表示 Extrinsic X→Y→Z，这里我们只给 Y
        q_rot = quaternion_from_euler(ry90, ry90, 0, axes='rxyz')
        # 对每一帧的四元数都做转换
        quat_list = np.array([quaternion_multiply(q, q_rot) for q in quat_list])
        # trans_kinova_0, rot_kinova_0 = transform_pose_table_to_kinova((trans_list[0], quat_list[0]), shift_kinova_test, axis_mapping)
        trans_kinova_0, rot_kinova_0 = transform_A_to_B_tf(trans_list[0], quat_list[0])
        # 移动到轨迹起始位置
        # [-0.81029483, -0.13515251,  0.10532389,  0.56041321]
        arm.ee_move(trans_kinova_0, rot_kinova_0, asynchronous=False)

        # 测试waypoint
        # trans_kinova, quat_kinova = batch_transform_A_to_B_tf(trans_list, quat_list)
        # arm.ee_move(trans_kinova[0], quat_kinova[0], asynchronous=False)
        # R = Rotation.from_quat(quat_kinova)
        # euler_list = R.as_euler('xyz', degrees=True)
        # waypoints = []
        # waypoints.append((trans_kinova[0][0], trans_kinova[0][1], trans_kinova[0][2], 0.0, euler_list[0][0], euler_list[0][1], euler_list[0][2]))
        # waypoints.append((trans_kinova[40][0], trans_kinova[40][1], trans_kinova[40][2], 0.0, euler_list[40][0], euler_list[40][1], euler_list[40][2]))
        # waypoints.append((trans_kinova[80][0], trans_kinova[80][1], trans_kinova[80][2], 0.0, euler_list[80][0], euler_list[80][1], euler_list[80][2]))
        # waypoints.append((trans_kinova[-1][0], trans_kinova[-1][1], trans_kinova[-1][2], 0.0, euler_list[-1][0], euler_list[-1][1], euler_list[-1][2]))
        # # for i in range(len(euler_list)):
        # #     waypoints.append((trans_kinova[i][0], trans_kinova[i][1], trans_kinova[i][2], 0.0, euler_list[i][0], euler_list[i][1], euler_list[i][2]))
        # arm.set_ee_trajectory(waypoints, optimize=False, asynchronous=False)

        pose_now = arm.get_ee_pose()
        print(pose_now)
        # euler_list = R.as_euler('xyz', degrees=True)
        # waypoints = []
        # for i in range(len(euler_list)):
        #     waypoints.append((trans_kinova_0[0], trans_kinova_0[1], trans_kinova_0[2], 0.1, euler_list[i][0], euler_list[i][1], euler_list[i][2]))
        # arm.set_ee_trajectory(waypoints, optimize=True, asynchronous=True)
        # pose_now = arm.get_ee_pose()
        # print(pose_now)
        input("Press Enter to start the trajectory playback...")
        
        # 轨迹偏移（可选，用于调整位置）
        
        if args.arm == 'kinova':
            # Kinova机械臂：需要完整的坐标系变换
            
            # 逐帧执行轨迹（应用完整变换）
            for trans, rot, grip in zip(trans_list, quat_list, gripper_list):
                # 将table坐标系的位姿转换到kinova坐标系
                trans_kinova, rot_kinova = transform_A_to_B_tf(trans, rot)
                arm.ee_move(trans_kinova, rot_kinova, asynchronous=False)
                # arm.set_gripper_opening(grip)  # 可选：同步夹爪状态
                rate.sleep()
            rate.sleep()
        else:
            # Franka机械臂：使用简单的平移变换
            for trans, rot, grip in zip(trans_list, quat_list, gripper_list):
                arm.ee_move(trans+shift_franka, rot, asynchronous=False)
                # arm.set_gripper_opening(grip)  # 可选：同步夹爪状态
                rate.sleep()
            # arm.set_gripper_opening(grip)  # 可选：同步夹爪状态
            rate.sleep()
