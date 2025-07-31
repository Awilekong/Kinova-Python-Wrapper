#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped, PoseStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
from tf.transformations import quaternion_from_euler, quaternion_multiply
import math
import numpy as np
import geometry_msgs.msg
import tf

def make_axis_marker(frame_id, marker_id, scale, color, timeout=0.0):
    """生成一个沿 X 轴的箭头 Marker，其他轴通过旋转 frame_id 实现。"""
    m = Marker()
    m.header = Header(frame_id=frame_id, stamp=rospy.Time.now())
    m.ns = "ee_axes"
    m.id = marker_id
    m.type = Marker.ARROW
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0  # 箭头默认沿 X 轴
    m.scale.x = scale      # 箭头杆长
    m.scale.y = scale*0.1  # 箭头杆粗
    m.scale.z = scale*0.1  # 箭头头粗
    m.color = ColorRGBA(*color)
    m.lifetime = rospy.Duration(timeout)
    return m

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
        print('p_B', p_B)
        print('q_B', q_B)
        return p_B, q_B
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logwarn("Transform failed")
        return None, None

def publish_static_transform():
    """发布静态TF变换：从table到kinova_base"""
    static_broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transformStamped = geometry_msgs.msg.TransformStamped()

    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "table"
    static_transformStamped.child_frame_id = "kinova_base"

    static_transformStamped.transform.translation.x = 1.01-0.025
    static_transformStamped.transform.translation.y = 0.3875
    static_transformStamped.transform.translation.z = 0.042

    static_transformStamped.transform.rotation.x = 0.0
    static_transformStamped.transform.rotation.y = 0.0
    static_transformStamped.transform.rotation.z = 1.0
    static_transformStamped.transform.rotation.w = 0.0

    static_broadcaster.sendTransform(static_transformStamped)
    return static_broadcaster  # 返回广播器对象以保持其生命周期

if __name__ == "__main__":
    rospy.init_node("ee_pose_visualizer")

    static_broadcaster = publish_static_transform()
    rospy.sleep(1.0)  # 等待TF变换建立

    # —— 参数化你的目标末端位姿 —— 
    # 可以通过 ROS 参数、service 调用、直接改下面的数值等方式传入
    # target_pos = rospy.get_param("~position", [0.39564483, 0.52938525, 0.56558619])
    # target_pos = rospy.get_param("~position", [0.58978486, -0.04194625,  0.52358619])
    # target_quat = rospy.get_param("~orientation", [-0.1352738 ,  0.8102906 ,  0.56048334,  -0.10532])
    target_pos = rospy.get_param("~position", [0.49719363, 0.28443663, 0.46399614])
    target_quat = rospy.get_param("~orientation", [-0.04681407,  0.72486689,  0.68729552,  0.00114685])
    # target_quat = rospy.get_param("~orientation", [0.1100265 ,  0.13985512, -0.56511582,  0.80559222])
    # 90° 转成弧度
    ry90 = math.radians(90)
    # axes='sxyz' 表示 Extrinsic X→Y→Z，这里我们只给 Y
    q_rot = quaternion_from_euler(ry90, ry90, 0, axes='rxyz')
    q_new = quaternion_multiply(target_quat, q_rot)
    # target_quat2 = [-0.13985512,  0.1100265 ,  0.80559222,  0.56511582]
    # target_rpy = rospy.get_param("~rpy", [0.02049988629960849, 0.3572378750451595, 1.9218418131992745])tt
    # target_rpy = rospy.get_param("~rpy", [1.17455697,  20.46822252, 110.11342479])
    parent_frame = rospy.get_param("~parent_frame", "base_link")
    ee_frame     = rospy.get_param("~ee_frame", "ee_target")
    # qx, qy, qz, qw = quaternion_from_euler(target_rpy[0], target_rpy[1], target_rpy[2], axes='sxyz')

    # TF broadcaster
    tf_broadcaster = tf2_ros.TransformBroadcaster()
    tf_msg = TransformStamped()
    tf_msg.header.frame_id = parent_frame
    tf_msg.child_frame_id  = ee_frame
    tf_msg.transform.translation.x = target_pos[0]
    tf_msg.transform.translation.y = target_pos[1]
    tf_msg.transform.translation.z = target_pos[2]
    tf_msg.transform.rotation.x    = target_quat[0]
    tf_msg.transform.rotation.y    = target_quat[1]
    tf_msg.transform.rotation.z    = target_quat[2]
    tf_msg.transform.rotation.w    = target_quat[3]

    tf_msg2 = TransformStamped()
    tf_msg2.header.frame_id = parent_frame
    tf_msg2.child_frame_id  = "ee_target2"
    tf_msg2.transform.translation.x = target_pos[0]
    tf_msg2.transform.translation.y = target_pos[1]
    tf_msg2.transform.translation.z = target_pos[2]
    tf_msg2.transform.rotation.x    = q_new[0]
    tf_msg2.transform.rotation.y    = q_new[1]
    tf_msg2.transform.rotation.z    = q_new[2]
    tf_msg2.transform.rotation.w    = q_new[3]
    
    trans_kinova_0, rot_kinova_0 = transform_A_to_B_tf(target_pos, q_new)
    tf_msg3 = TransformStamped()
    tf_msg3.header.frame_id = parent_frame
    tf_msg3.child_frame_id  = "ee_target3"
    tf_msg3.transform.translation.x = trans_kinova_0[0]
    tf_msg3.transform.translation.y = trans_kinova_0[1]
    tf_msg3.transform.translation.z = trans_kinova_0[2]
    tf_msg3.transform.rotation.x    = rot_kinova_0[0]
    tf_msg3.transform.rotation.y    = rot_kinova_0[1]
    tf_msg3.transform.rotation.z    = rot_kinova_0[2]
    tf_msg3.transform.rotation.w    = rot_kinova_0[3]

    # Marker 发布者
    marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=1)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # 更新时间戳并广播 TF
        # tf_msg.header.stamp = rospy.Time.now()
        # tf_broadcaster.sendTransform(tf_msg)
        # tf_msg2.header.stamp = rospy.Time.now()
        # tf_broadcaster.sendTransform(tf_msg2)
        tf_msg.header.stamp = rospy.Time.now()
        tf_broadcaster.sendTransform(tf_msg)
        # 发布三个坐标轴箭头
        # 1) X 轴：红色
        m_x = make_axis_marker(ee_frame, marker_id=0, scale=0.1, color=(1,0,0,1))
        # 2) Y 轴：绿 → 沿 Z 轴旋转 +90°
        m_y = make_axis_marker(ee_frame, marker_id=1, scale=0.1, color=(0,1,0,1))
        m_y.pose.orientation.z = 0.7071
        m_y.pose.orientation.w = 0.7071
        # 3) Z 轴：蓝 → 沿 Y 轴旋转 –90°
        m_z = make_axis_marker(ee_frame, marker_id=2, scale=0.1, color=(0,0,1,1))
        m_z.pose.orientation.y = -0.7071
        m_z.pose.orientation.w = 0.7071

        for m in (m_x, m_y, m_z):
            # 重要：更新时间戳
            m.header.stamp = rospy.Time.now()
            marker_pub.publish(m)
            print(1)

        rate.sleep()
