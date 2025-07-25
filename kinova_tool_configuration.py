#!/usr/bin/env python3

"""
kinova_tool_configuration.py
----------------------------
Kinova机械臂工具配置示例
展示如何为自定义夹爪设置正确的工具变换参数
"""

import sys
import os
import numpy as np
from scipy.spatial.transform import Rotation

# 导入Kinova类
from kinova_basic import Kinova

class KinovaToolConfiguration:
    """Kinova工具配置管理类"""
    
    def __init__(self, kinova):
        """
        初始化工具配置管理器
        
        参数：
            kinova: Kinova机械臂对象
        """
        self.kinova = kinova
        
    def set_default_gripper_config(self):
        """设置默认夹爪配置（Robotiq 2-Finger 85）"""
        try:
            # 导入ControlConfig相关模块
            from kortex_api.autogen.messages import ControlConfig_pb2
            
            # 创建工具配置
            tool_config = ControlConfig_pb2.ToolConfiguration()
            
            # 设置工具变换（相对于末端接口模块）
            # Robotiq 2-Finger 85的典型参数
            tool_config.tool_transform.x = 0.0      # 米
            tool_config.tool_transform.y = 0.0      # 米  
            tool_config.tool_transform.z = 0.1643    # 85mm偏移
            # tool_config.tool_transform.z = 0.45    # 85mm偏移
            tool_config.tool_transform.theta_x = 0.0  # 度
            tool_config.tool_transform.theta_y = 0.0  # 度
            tool_config.tool_transform.theta_z = 0.0  # 度
            
            # 设置工具质量
            tool_config.tool_mass = 1.2  # 千克
            # tool_config.tool_mass = 1.8  # 千克
            
            # 设置工具质心位置
            tool_config.tool_mass_center.x = 0.0
            tool_config.tool_mass_center.y = 0.0
            tool_config.tool_mass_center.z = 0.0821  # 质心在夹爪中心
            
            # 应用配置
            self.kinova.control_config.SetToolConfiguration(tool_config)
            print("✓ 默认夹爪配置设置成功")
            return True
            
        except Exception as e:
            print(f"✗ 默认夹爪配置设置失败: {e}")
            return False
    
    def set_custom_gripper_config(self, transform_params, mass=0.5, mass_center=None):
        """
        设置自定义夹爪配置
        
        参数：
            transform_params (dict): 变换参数字典
                - x, y, z: 位置偏移（米）
                - theta_x, theta_y, theta_z: 姿态偏移（度）
            mass (float): 夹爪质量（千克）
            mass_center (dict): 质心位置（米），None表示使用默认值
        """
        try:
            from kortex_api.autogen.messages import ControlConfig_pb2
            
            # 创建工具配置
            tool_config = ControlConfig_pb2.ToolConfiguration()
            
            # 设置工具变换
            tool_config.tool_transform.x = transform_params.get('x', 0.0)
            tool_config.tool_transform.y = transform_params.get('y', 0.0)
            tool_config.tool_transform.z = transform_params.get('z', 0.0)
            tool_config.tool_transform.theta_x = transform_params.get('theta_x', 0.0)
            tool_config.tool_transform.theta_y = transform_params.get('theta_y', 0.0)
            tool_config.tool_transform.theta_z = transform_params.get('theta_z', 0.0)
            
            # 设置工具质量
            tool_config.tool_mass = mass
            
            # 设置工具质心位置
            if mass_center is None:
                # 默认质心在夹爪几何中心
                tool_config.tool_mass_center.x = transform_params.get('x', 0.0)
                tool_config.tool_mass_center.y = transform_params.get('y', 0.0)
                tool_config.tool_mass_center.z = transform_params.get('z', 0.0) / 2.0
            else:
                tool_config.tool_mass_center.x = mass_center.get('x', 0.0)
                tool_config.tool_mass_center.y = mass_center.get('y', 0.0)
                tool_config.tool_mass_center.z = mass_center.get('z', 0.0)
            
            # 应用配置
            self.kinova.control_config.SetToolConfiguration(tool_config)
            print("✓ 自定义夹爪配置设置成功")
            return True
            
        except Exception as e:
            print(f"✗ 自定义夹爪配置设置失败: {e}")
            return False
    
    def get_current_tool_config(self):
        """获取当前工具配置"""
        try:
            tool_config = self.kinova.control_config.GetToolConfiguration()
            
            print("当前工具配置:")
            print(f"  位置偏移: ({tool_config.tool_transform.x:.3f}, "
                  f"{tool_config.tool_transform.y:.3f}, {tool_config.tool_transform.z:.3f}) 米")
            print(f"  姿态偏移: ({tool_config.tool_transform.theta_x:.1f}, "
                  f"{tool_config.tool_transform.theta_y:.1f}, {tool_config.tool_transform.theta_z:.1f}) 度")
            print(f"  工具质量: {tool_config.tool_mass:.3f} 千克")
            print(f"  质心位置: ({tool_config.tool_mass_center.x:.3f}, "
                  f"{tool_config.tool_mass_center.y:.3f}, {tool_config.tool_mass_center.z:.3f}) 米")
            
            return tool_config
            
        except Exception as e:
            print(f"✗ 获取工具配置失败: {e}")
            return None
    
    def reset_tool_config(self):
        """重置工具配置为默认值"""
        try:
            tool_config = self.kinova.control_config.ResetToolConfiguration()
            print("✓ 工具配置已重置为默认值")
            return True
        except Exception as e:
            print(f"✗ 重置工具配置失败: {e}")
            return False

def example_tool_configuration():
    """工具配置示例"""
    print("=== Kinova工具配置示例 ===\n")
    
    # 创建Kinova对象
    try:
        kinova = Kinova(robot_ip="192.168.1.10")
        print("✓ Kinova机械臂连接成功")
    except Exception as e:
        print(f"✗ Kinova机械臂连接失败: {e}")
        return
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          
    # 创建工具配置管理器
    tool_config = KinovaToolConfiguration(kinova)
    
    try:
        # 1. 查看当前工具配置
        print("\n1. 查看当前工具配置:")
        current_config = tool_config.get_current_tool_config()
        
        # 2. 设置默认夹爪配置
        print("\n2. 设置默认夹爪配置:")
        tool_config.set_default_gripper_config()
        
        # 3. 设置自定义夹爪配置示例
        print("\n3. 设置自定义夹爪配置示例:")
        
        # 示例：一个向前偏移50mm，向下偏移30mm的自定义夹爪
        custom_transform = {
            'x': 0.0,      # 无X方向偏移
            'y': 0.0,      # 无Y方向偏移  
            'z': 0.080,    # 向前偏移80mm
            'theta_x': 0.0, # 无X轴旋转
            'theta_y': 0.0, # 无Y轴旋转
            'theta_z': 0.0  # 无Z轴旋转
        }
        
        tool_config.set_custom_gripper_config(
            transform_params=custom_transform,
            mass=0.3,  # 300g质量
            mass_center={'x': 0.0, 'y': 0.0, 'z': 0.040}  # 质心在夹爪中心
        )
        
        # 4. 验证配置效果
        print("\n4. 验证配置效果:")
        
        # 获取当前关节角度
        current_joints = kinova.get_joint_pose()
        print(f"当前关节角度: {current_joints}")
        
        # 计算正运动学（使用新配置）
        pose = kinova.compute_fk(current_joints)
        print(f"正运动学结果: 位置={pose[:3]}, 姿态={pose[3:]}")
        
        # 计算逆运动学（使用新配置）
        target_pose = pose  # 使用当前位姿作为目标
        ik_solution = kinova.compute_ik(target_pose, guess=current_joints)
        if ik_solution is not None:
            print(f"逆运动学结果: {ik_solution}")
            print("✓ IK/FK计算正常")
        else:
            print("✗ 逆运动学求解失败")
        
        # 5. 重置为默认配置
        print("\n5. 重置为默认配置:")
        tool_config.reset_tool_config()
        
        print("\n=== 工具配置示例完成 ===")
        
    except Exception as e:
        print(f"✗ 示例执行过程中发生错误: {e}")
    
    finally:
        # 确保连接正确关闭
        kinova.close()
        print("✓ 连接已关闭")

def measure_gripper_parameters():
    """
    测量夹爪参数的实用函数
    用于确定自定义夹爪的准确变换参数
    """
    print("\n=== 夹爪参数测量指南 ===\n")
    
    print("要准确配置自定义夹爪，需要测量以下参数：")
    print("\n1. 几何参数测量:")
    print("   - 夹爪安装面到夹爪中心的距离（Z方向偏移）")
    print("   - 夹爪中心相对于末端接口的X、Y偏移")
    print("   - 夹爪的安装角度（如果有旋转）")
    
    print("\n2. 质量参数测量:")
    print("   - 夹爪总质量")
    print("   - 夹爪质心位置")
    
    print("\n3. 测量方法:")
    print("   - 使用卡尺测量几何尺寸")
    print("   - 使用天平测量质量")
    print("   - 使用平衡法确定质心位置")
    
    print("\n4. 配置示例:")
    print("   custom_transform = {")
    print("       'x': 0.0,      # X方向偏移（米）")
    print("       'y': 0.0,      # Y方向偏移（米）")
    print("       'z': 0.080,    # Z方向偏移（米）")
    print("       'theta_x': 0.0, # X轴旋转（度）")
    print("       'theta_y': 0.0, # Y轴旋转（度）")
    print("       'theta_z': 0.0  # Z轴旋转（度）")
    print("   }")

if __name__ == "__main__":
    # 运行工具配置示例
    example_tool_configuration()
    
    # 显示测量指南
    measure_gripper_parameters() 