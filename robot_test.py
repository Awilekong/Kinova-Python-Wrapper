from kinova_basic import Kinova
import numpy as np
from scipy.spatial.transform import Rotation
arm = Kinova()
# joint_up = np.array([-0.97788733, -1.04903993,  1.31520369, -1.58949637, -0.26875838,  1.36971498, 2.23423306])
# joint_down = np.array([-1.12243027, -1.2869527, 1.72586445, -2.25379698,  0.18903419, 2.15440121, 2.43160574])

# # 移动到预设位置
# arm.set_joint_pose(joint_up, asynchronous=False)

# pos, quat, euler = arm.get_ee_pose()
# pose = list(pos) + list(euler)
# print("pose:",pose)
# joint_angles_deg = arm.get_joint_pose()
# print("joint_angles_deg", joint_angles_deg)
# # angles_rad = [np.deg2rad(a) for a in joint_angles_deg]
# # print(angles_rad)
# # ee = arm.compute_fk(angles_rad)
# # print("fk:",ee)
# # print()
# joint_angles = arm.compute_ik(pose, seed = joint_angles_deg)
# joint_angles1 = [angles+180 for angles in joint_angles]
# print("ik:",joint_angles)
# arm.set_joint_pose(joint_angles1)
# ee1 = arm.compute_fk(joint_angles)

# print(ee1)
joint = [145.24061748, 291.63036466, 76.32623881, 53.0930225, 64.71266326, 267.38429277, 175.81600754]
# joint = [q+180 for q in joint]
# arm.set_joint_pose(joint)

ee = [0.3, -0.1, 0.3]
quat = [0.7071068, 0, 0, 0.7071068]
# arm.ee_move(ee, quat)
pose, quat, euler = arm.get_ee_pose()
joint_angles_deg = arm.get_joint_pose()
print("joint_angles_deg", joint_angles_deg)
print(pose)
print(quat)
print(euler)
# pos = arm.get_ee_pose()
# print(pos)