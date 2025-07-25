#!/usr/bin/env python3
"""
franka_data_recording.py
------------------------
Franka机械臂数据录制脚本

功能：
1. 录制人类演示的轨迹数据（位置、姿态、夹爪状态）
2. 同步录制RGB和深度图像
3. 通过ArUco标记检测夹爪状态
4. 通过VICON系统获取人类手部位置
5. 将数据保存为HDF5格式

主要组件：
- RealSense相机：获取RGB和深度图像
- VICON系统：获取人类手部位置和姿态
- ArUco标记：检测夹爪开合状态
- ROS TF：处理坐标变换
"""

import numpy as np
import pyrealsense2 as rs
import os, rospy, signal, argparse
import tf, cv2, collections, h5py
from scipy.spatial.transform import Rotation
from utils import ARUCO_DICT, aruco_display, get_center
# from franka_robot.franka_dual_arm import FrankaLeft, FrankaRight
from scipy.spatial.transform import Rotation as R

# 配置参数
DROP_FRAME_NUM = 10  # 丢弃开始和结束的帧数，避免不稳定数据
LEFT_CAM_ID = '315122271073'  # 左相机序列号
RIGHT_CAM_ID = '419122270338'  # 右相机序列号

def publish_static_tf(broadcaster):
    """
    发布静态坐标变换：从franka_table到franka_base
    
    参数：
        broadcaster: ROS TF广播器
    """
    broadcaster.sendTransform(
        (-1.0+0.025, 0.0, 0.015),  # 平移向量（米）
        (0.0, 0.0, 0.0, 1.0),      # 四元数（无旋转）
        rospy.Time.now(),
        "franka_base",              # 子坐标系
        "/vicon/franka_table/franka_table"  # 父坐标系
    )

def get_rl_pipeline(selected_serial):
    """
    初始化RealSense相机管道
    
    参数：
        selected_serial (str): 相机序列号
    返回：
        pipeline: RealSense管道对象
        align: 深度图像对齐对象
    """
    # 相机初始化
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(selected_serial)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)  # RGB流
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)    # 深度流
    pipeline.start(config)
    align = rs.align(rs.stream.color)  # 将深度图像对齐到RGB图像

    return pipeline, align

def get_RGBframe(pipeline):
    """
    获取RGB图像帧
    
    参数：
        pipeline: RealSense管道
    返回：
        color: BGR格式的图像，失败时返回None
    """
    frames = pipeline.wait_for_frames()
    color = frames.get_color_frame()
    if not color: 
        return None
    else:
        color_np = np.asanyarray(color.get_data())
        color = cv2.cvtColor(color_np, cv2.COLOR_RGB2BGR)  # RGB转BGR
        return color

def get_RGBDframe(rs_pl):
    """
    获取RGB和深度图像帧
    
    参数：
        rs_pl: (pipeline, align)元组
    返回：
        color: BGR格式的RGB图像
        depth_np: 深度图像数组（毫米）
    """
    pipeline, align = rs_pl
    frames = pipeline.wait_for_frames()
    frames = align.process(frames)  # 将深度图像对齐到RGB图像

    color_frame = frames.get_color_frame()
    depth_frame = frames.get_depth_frame()

    if not color_frame or not depth_frame:
        return None, None
    else:
        color_np = np.asanyarray(color_frame.get_data())
        color = cv2.cvtColor(color_np, cv2.COLOR_RGB2BGR)

        depth_np = np.asanyarray(depth_frame.get_data())

        # 可视化相机图像
        depth_visual = cv2.convertScaleAbs(depth_np, alpha=0.03)  # 深度图像可视化
        cv2.imshow("Depth Image", depth_visual)
        cv2.imshow("RGB Image", color)
        cv2.waitKey(1)

        return color, depth_np

def get_gripper_state(rgb, detector, gripper_state, marker_left, marker_right):
    """
    通过ArUco标记检测夹爪状态
    
    参数：
        rgb: RGB图像
        detector: ArUco检测器
        gripper_state: 当前夹爪状态
        marker_left: 左标记位置
        marker_right: 右标记位置
    返回：
        gripper_state: 更新后的夹爪状态（0.0=闭合，0.05=张开）
        marker_left: 左标记位置
        marker_right: 右标记位置
        event: 是否检测到事件（标记消失）
    """
    gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected_img_points = detector.detectMarkers(gray)
    event = False

    if len(corners) > 1:
        for corner, id in zip(corners, ids):
            cx, cy = get_center(corner)
            if id == 0:  # 左标记
                marker_left = cx
            if id == 1:  # 右标记
                marker_right = cx
    elif len(corners) == 0:
        event = True  # 标记消失，触发事件

    # 根据标记间距判断夹爪状态
    if marker_left is not None and marker_right is not None:
        pix_diff = abs(marker_right-marker_left)
        if pix_diff < 200:  # 像素距离小于200，认为夹爪闭合
            gripper_state = 0.0
        else:  # 否则认为夹爪张开
            gripper_state = 0.05

    return gripper_state, marker_left, marker_right, event

def signal_handler(sig, frame):
    """Ctrl+C信号处理函数"""
    print('You pressed Ctrl+C!')
    rospy.signal_shutdown("You pressed Ctrl+C!")

def get_file_path(dir_path):
    """
    生成文件保存路径
    
    参数：
        dir_path: 目录路径
    返回：
        file_path: 完整的文件路径
    """
    if not os.path.exists(dir_path):
        os.makedirs(dir_path)
    f_list = os.listdir(dir_path)
    file_path = dir_path + '/' + str(len(f_list)) + '.h5'  # 按序号命名
    print('get_file_path', file_path)
    return file_path

def init_devices(camera_name):
    """
    初始化设备（相机和ArUco检测器）
    
    参数：
        camera_name: 相机名称（'left'或'right'）
    返回：
        pipeline: RealSense管道
        detector: ArUco检测器
    """
    if 'left' in camera_name:
        cam_id = LEFT_CAM_ID
    else:
        cam_id = RIGHT_CAM_ID

    # 相机初始化
    pipeline = get_rl_pipeline(cam_id)

    # ArUco标记相关初始化
    arucoDict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT['DICT_4X4_50'])
    arucoParams = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)    
    return pipeline, detector
    
def save_data(file_path):
    """
    保存数据到HDF5文件
    
    参数：
        file_path: 保存路径
    """
    file_path = get_file_path(file_path)
    with h5py.File(file_path, 'w') as hdf5_file:
        for key, value in data.items():
            # 丢弃开始和结束的不稳定帧
            value = value[DROP_FRAME_NUM:-DROP_FRAME_NUM]
            hdf5_file.create_dataset(key, data=value)
            print(key, len(value))
        print('----- save to', file_path, len(data['translation']))

def get_trajectory(path, idx=0):
    """
    获取轨迹数据
    
    参数：
        path: 数据路径
        idx: 文件索引
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

def add_text(img, zip_pos):
    """
    在图像上添加拉链位置文本
    
    参数：
        img: 输入图像
        zip_pos: 拉链位置文本
    """
    # 文本设置
    text = zip_pos
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 1
    thickness = 2
    color = (255, 0, 0)  # 红色

    # 获取文本尺寸
    (text_width, text_height), baseline = cv2.getTextSize(text, font, font_scale, thickness)

    # 计算右下角位置
    x = img.shape[1] - text_width - 10  # 距离右边10像素
    y = img.shape[0] - 10  # 距离底部10像素

    # 添加文本
    cv2.putText(img, text, (x, y), font, font_scale, color, thickness, cv2.LINE_AA)

    # 显示结果
    cv2.imshow("Image with Bottom-Right Text", img)
    cv2.waitKey(1)

def get_euler_difference(quat1, quat2):
    """
    计算两个四元数之间的欧拉角差异
    
    参数：
        quat1: 第一个四元数
        quat2: 第二个四元数
    返回：
        angle_deg: 角度差异（度）
    """
    r1 = R.from_quat(quat1)
    r2 = R.from_quat(quat2)

    # 计算相对旋转：r_rel应用到quat1得到quat2
    r_rel = r2 * r1.inv()

    angle_rad = r_rel.magnitude()
    angle_deg = np.degrees(angle_rad)

    return angle_deg 

if __name__ == '__main__':
    # ROS和系统相关初始化
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node("franka_data_recording")
    broadcaster = tf.TransformBroadcaster()
    listener = tf.TransformListener()
    rate = rospy.Rate(10)  # 10Hz循环频率

    # 录制相关初始化
    parser = argparse.ArgumentParser()
    parser.add_argument('--arm', default='left', type=str)        # 机械臂：left, right        
    parser.add_argument('--cam', default='cam_up', type=str)      # 相机：up, down（初始夹爪位置）
    parser.add_argument('--zip', default='zip_top', type=str)     # 拉链：top, bottom（拉链位置）
    parser.add_argument('--item', default='small_box', type=str)  # 物品类型
    parser.add_argument('--data_mode', default='grasp', type=str) # 数据模式：grasp, open, grasp_noise, open_noise
    parser.add_argument('--base_path', default='./paper_hdf5_v4/human', type=str)  # 数据保存路径
    args = parser.parse_args()

    # 创建保存路径
    human_traj_save_path = os.path.join(args.base_path, args.data_mode, args.item, args.zip, args.cam)
    if not os.path.exists(human_traj_save_path):
        os.makedirs(human_traj_save_path)    

    # 初始化设备
    pipeline, detector = init_devices(args.arm)

    # 主循环
    data = {'rgb':[], 'depth':[], 'translation':[], 'rotation':[], 'gripper_w':[]}  # 数据存储字典
    event_list = collections.deque(maxlen=5)  # 事件队列，用于检测稳定的触发条件
    marker_left, marker_right = None, None
    gripper_state = 0.04  # 初始夹爪状态（张开）
    start, event_tirgger, ready = False, False, False  # 录制状态标志

    g_count = 0  # 录制计数器
    pre_trans, pre_quat = None, None  # 前一帧的位置和姿态

    while not rospy.is_shutdown():
        # 发布静态坐标变换
        publish_static_tf(broadcaster)

        # 获取RGB和深度图像
        rgb, depth = get_RGBDframe(pipeline)
        if rgb is None:
            continue

        # 检测夹爪状态
        gripper_state, marker_left, marker_right, event = get_gripper_state(rgb, detector, gripper_state, marker_left, marker_right)
        
        # 获取人类手部位置（通过VICON系统）
        try:
            (human_trans, human_rot) = listener.lookupTransform('franka_table', '/vicon/franka_human/franka_human', rospy.Time(0))
            (trans_w, rot_w) = listener.lookupTransform('franka_table', '/vicon/franka_human/franka_human', rospy.Time(0))
        ## =======================修改==============
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Failed to lookup transform from gripper to franka_base.")
            continue

        # 事件检测逻辑
        event_list.append(event)
        event_tirgger = len(event_list) == 5 and all(event_list)  # 检查最近5帧是否都为True
        
        if event_tirgger:
            ready = True

        # 录制控制逻辑
        if ready and not event_tirgger and not start:  # 触发开始录制
            start = True
            ready = False
            print('start recording')

            # 交替设置拉链位置（top/bottom）
            if g_count % 2 == 0:
                zip_pos = 'zip_bottom'
            else:
                zip_pos = 'zip_top'
            human_traj_save_path = os.path.join(args.base_path, args.data_mode, args.item, zip_pos, args.cam)
            if not os.path.exists(human_traj_save_path):
                os.makedirs(human_traj_save_path)

            add_text(rgb, zip_pos)
            print('----------- current zip pose', zip_pos)
            
        elif event_tirgger and start:  # 触发跳过帧（标记消失时）
            print('skip frame', len(data['gripper_w']))
            continue
            
        elif ready and not event_tirgger and start:  # 触发停止录制
            start = False
            ready = False
            print('stop recording', len(data['gripper_w']))

            save_data(human_traj_save_path)
            data = {'rgb':[], 'depth':[], 'translation':[], 'rotation':[], 'gripper_w':[]}  # 重置数据
            g_count += 1
            pre_trans, pre_quat = None, None

        # 数据录制
        if start:
            if pre_trans is None:
                pre_trans, pre_quat = human_trans, human_rot

            # 计算位置和姿态变化
            trans_diff_pre = np.linalg.norm(np.array(human_trans) - np.array(pre_trans))
            euler_diff_pre = get_euler_difference(human_rot, pre_quat)

            # 过滤异常跳变（位置变化>5cm或角度变化>15度）
            if trans_diff_pre > 0.05 or euler_diff_pre > 15.0:
                print('!!!!!!!!!!!!!!!!!!! skip frame jump', trans_diff_pre, euler_diff_pre)
                continue

            # 保存数据
            data['translation'].append(human_trans)
            data['rotation'].append(human_rot)
            data['translation_w'].append(trans_w)
            data['rotation_w'].append(rot_w)
            data['gripper_w'].append(gripper_state)
            data['rgb'].append(rgb)
            data['depth'].append(depth*1)

            pre_trans, pre_quat = human_trans, human_rot

        rate.sleep()

# 注释：相机标定数据示例
# cam_top
# [0.36638517 0.30260678 0.57729757]
# degree: [ 0.03939009  0.30290007 -1.27132275] [  2.25688617  17.35489556 -72.84142787]
# quat [ 0.10522243  0.10982125 -0.58919091  0.79355   ]
# Joints:  [-0.97788733, -1.04903993,  1.31520369, -1.58949637, -0.26875838,  1.36971498, 2.23423306]
# Elbow:  ElbowState(joint_3_pos=1.3152, joint_4_flip=FlipDirection.Negative)