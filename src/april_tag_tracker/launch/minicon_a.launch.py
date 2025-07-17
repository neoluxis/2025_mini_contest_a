#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取april_tag_tracker包的路径
    april_tag_tracker_path = get_package_share_directory('april_tag_tracker')
    
    # 创建april_tag_tracker节点
    april_tag_tracker_node = Node(
        package='april_tag_tracker',
        executable='april_tag_tracker',
        name='april_tag_tracker',
        output='screen',
        parameters=[{
            'use_sim_time': False,
        }]
    )
    
    # 创建serial节点
    serial_node = Node(
        package='april_tag_tracker',
        executable='serial',
        name='serial_sender',
        output='screen',
        parameters=[{
            'use_sim_time': False,
        }]
    )
    
    # 包含hobot_usb_cam的websocket launch文件
    try:
        hobot_usb_cam_path = get_package_share_directory('hobot_usb_cam')
        hobot_usb_cam_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(hobot_usb_cam_path, 'launch', 'hobot_usb_cam_websocket.launch.py')
            ]),
            launch_arguments={
                'device': '/dev/video0'
            }.items()
        )
    except Exception as e:
        print(f"Warning: Cannot find hobot_usb_cam package: {e}")
        # 如果找不到包，可以使用ExecuteProcess作为备选方案
        hobot_usb_cam_launch = ExecuteProcess(
            cmd=['ros2', 'launch', 'hobot_usb_cam', 'hobot_usb_cam_websocket.launch.py', 'device:=/dev/video0'],
            output='screen'
        )
    
    return LaunchDescription([
        april_tag_tracker_node,
        serial_node,
        hobot_usb_cam_launch,
    ])