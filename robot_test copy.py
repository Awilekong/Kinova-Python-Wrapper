from kinova_basic import Kinova
import numpy as np
from scipy.spatial.transform import Rotation
arm = Kinova()
# joint_up = np.array([-0.97788733, -1.04903993,  1.31520369, -1.58949637, -0.26875838,  1.36971498, 2.23423306])
# joint_down = np.array([-1.12243027, -1.2869527, 1.72586445, -2.25379698,  0.18903419, 2.15440121, 2.43160574])

# # 移动到预设位置
# arm.set_joint_pose(joint_up, asynchronous=False)

xyz, quat, rpy = arm.get_ee_pose()
# # euler = Rotation.from_quat(quat).as_euler('xyz', degrees=True)
# # print("euler:",euler,"rpy:",rpy)
# joint_angles_deg = arm.get_joint_pose()
# print("now",joint_angles_deg)
# pose = [xyz[0], xyz[1], xyz[2], rpy[0], rpy[1], rpy[2]]
# joint_angles = arm.ik(pose, joint_angles_deg)
# print("ik:",joint_angles)


xyz = [0.552, 0.196, 0.508]
quat = [0.02174881, 0.93459244, 0.35307589, 0.03743484]

# print(pos)
# xyz[2]=xyz[2]+0.1
# pose = list(pos) + list(euler)
# print("pose:",pose)
pose = [xyz[0], xyz[1], xyz[2], quat[0], quat[1], quat[2], quat[3]]
joint_angles_deg = arm.get_joint_pose()
print("当前关节角", joint_angles_deg)
arm.set_ee_pose(xyz, quat)

# angles_rad = [np.deg2rad(a) for a in joint_angles_deg]
# print(angles_rad)
# ee = arm.compute_fk(angles_rad)
# print("fk:",ee)
# print()
# joint_angles = arm.compute_ik(pose, seed = joint_angles_deg)
# joint_angles = [angles+180 for angles in joint_angles]
# print("ik:",joint_angles)
# arm.set_joint_pose(joint_angles)
# ee1 = arm.compute_fk(joint_angles)

# print(ee1)
# pos = arm.get_ee_pose()
# print(pos)