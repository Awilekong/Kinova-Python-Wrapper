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
import rospy, tf
import signal
import os
import argparse
import cv2
import h5py
from franka_robot.franka_dual_arm import FrankaLeft, FrankaRight
from kinova_basic import Kinova

# from scipy.spatial.transform import Rotation

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
    基于轴映射创建变换矩阵
    
    参数：
        axis_mapping: 坐标轴映射字典
        translation: 平移向量（相对于table坐标系）
    返回：
        new_translation: 变换后的平移向量（相对于kinova坐标系）
        R: 旋转矩阵
    """
    # 根据轴映射创建旋转矩阵
    R = np.zeros((3, 3))
    
    # 解析轴映射并构建旋转矩阵
    for kinova_axis, table_axis in axis_mapping.items():
        # 确定Kinova轴的索引
        if kinova_axis == 'kinova_x':
            kinova_idx = 0
        elif kinova_axis == 'kinova_y':
            kinova_idx = 1
        elif kinova_axis == 'kinova_z':
            kinova_idx = 2
        else:
            continue
        
        # 根据table轴映射确定旋转矩阵的列
        if table_axis == 'table_x':
            R[:, kinova_idx] = [1, 0, 0]  # table的X轴
        elif table_axis == 'table_y':
            R[:, kinova_idx] = [0, 1, 0]  # table的Y轴
        elif table_axis == 'table_z':
            R[:, kinova_idx] = [0, 0, 1]  # table的Z轴
        elif table_axis == 'table_-x':
            R[:, kinova_idx] = [-1, 0, 0]  # table的负X轴
        elif table_axis == 'table_-y':
            R[:, kinova_idx] = [0, -1, 0]  # table的负Y轴
        elif table_axis == 'table_-z':
            R[:, kinova_idx] = [0, 0, -1]  # table的负Z轴
    
    # 验证旋转矩阵的正交性
    if not np.allclose(R @ R.T, np.eye(3), atol=1e-6):
        raise ValueError("生成的旋转矩阵不是正交矩阵，请检查轴映射")
    
    # 计算变换后的平移向量
    # 从table坐标系到kinova坐标系的平移变换
    new_translation = R.T @ translation  # 注意：这里用R.T而不是R
    
    return new_translation, R

def transform_pose_table_to_kinova(pose, translation, axis_mapping):
    """
    将位姿从table坐标系转换到kinova坐标系
    
    参数：
        pose: (trans, quat) 原始位姿
        translation: 平移向量（相对于table坐标系）
        axis_mapping: 轴映射字典
    返回：
        transformed_pose: (trans, quat) 变换后的位姿
    """
    trans, quat = pose
    
    # 获取变换矩阵
    new_translation, R = create_transform_matrix(axis_mapping, translation)
    
    # 变换位置
    trans_transformed = R.T @ trans + new_translation
    
    # 变换姿态
    from scipy.spatial.transform import Rotation
    R_original = Rotation.from_quat(quat)
    R_new = Rotation.from_matrix(R.T) * R_original
    quat_transformed = R_new.as_quat()
    
    return trans_transformed, quat_transformed


if __name__ == '__main__':
    # 系统初始化
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node("franka_data_generation")
    rate = rospy.Rate(10)  # 10Hz循环频率

    # 命令行参数解析
    parser = argparse.ArgumentParser()

    parser.add_argument('--base_path', default='./paper_hdf5_v4/human', type=str)  # 数据基础路径
    parser.add_argument('--arm', default='left', type=str)        # 机械臂：left, right        
    parser.add_argument('--cam', default='cam_up', type=str)      # 相机：up, down（初始夹爪位置）
    parser.add_argument('--zip', default='zip_top', type=str)     # 拉链：top, bottom（拉链位置）
    parser.add_argument('--item', default='small_box', type=str)  # 物品类型
    parser.add_argument('--data_mode', default='grasp', type=str) # 数据模式：grasp, open, grasp_noise, open_noise

    parser.add_argument('--idx', default=0, type=int)             # 文件索引
    parser.add_argument('--mode', default='rgb', type=str)        # 模式：rgb, depth, trajectory
    args = parser.parse_args()

    # 构建轨迹路径
    traj_path = os.path.join(args.base_path, args.data_mode, args.item, args.zip, args.cam)
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
        if 'left' in args.arm:
            arm = FrankaLeft() 
        elif 'right' in args.arm:
            arm = FrankaRight()
        else:
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
        arm.set_joint_pose(joint_up, asynchronous=False)
        
        # 等待用户确认开始回放
        input("Press Enter to start the trajectory playback...")

        # 获取轨迹数据
        data = get_trajectory(traj_path, idx=args.idx)
        trans_list, quat_list, gripper_list = np.array(data['translation']), np.array(data['rotation']), np.array(data['gripper_w'])

        # 移动到轨迹起始位置
        arm.set_ee_pose(trans_list[0], quat_list[0], asynchronous=False)
        input("Press Enter to start the trajectory playback...")
        
        # 轨迹偏移（可选，用于调整位置）
        shift_franka = np.array([-1.0+0.025, 0.0, 0.015])
        shift_kinova = np.array([0.01, 0.132, 0.0435])
        
        if args.arm == 'kinova':
            # Kinova机械臂：需要完整的坐标系变换
            axis_mapping = {
                'kinova_x': 'table_-x',    # Kinova的X轴 = table的负X轴
                'kinova_y': 'table_-y',    # Kinova的Y轴 = table的负Y轴
                'kinova_z': 'table_z'      # Kinova的Z轴 = table的Z轴
            }
            
            # 逐帧执行轨迹（应用完整变换）
            for trans, rot, grip in zip(trans_list, quat_list, gripper_list):
                # 将table坐标系的位姿转换到kinova坐标系
                trans_kinova, rot_kinova = transform_pose_table_to_kinova((trans, rot), shift_kinova, axis_mapping)
                arm.set_ee_pose(trans_kinova, rot_kinova, asynchronous=False)
                # arm.set_gripper_opening(grip)  # 可选：同步夹爪状态
                rate.sleep()
            rate.sleep()
        else:
            # Franka机械臂：使用简单的平移变换
            for trans, rot, grip in zip(trans_list, quat_list, gripper_list):
                arm.set_ee_pose(trans+shift_franka, rot, asynchronous=False)
                # arm.set_gripper_opening(grip)  # 可选：同步夹爪状态
                rate.sleep()
            # arm.set_gripper_opening(grip)  # 可选：同步夹爪状态
            rate.sleep()
