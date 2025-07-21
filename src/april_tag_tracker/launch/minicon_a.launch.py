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
    hobot_usb_cam_path = get_package_share_directory('hobot_usb_cam')
    hobot_usb_cam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(hobot_usb_cam_path, 'launch', 'hobot_usb_cam.launch.py')
        ]),
        launch_arguments={
            'device': '/dev/video0',
            # 'image_height': '240',
            # 'image_width': '320'
        }.items()
    )
    # cam_node = Node(
    #     package='april_tag_tracker',
    #     executable='videocap',
    #     name='videocap',
    #     output='screen',
    # )
    
    websocket_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('websocket') + '/launch/websocket.launch.py'
        ]),
        launch_arguments={
            'websocket_image_topic': '/image',
            'websocket_only_show_image': 'true',
        }.items()
    )
    
    return LaunchDescription([
        april_tag_tracker_node,
        # serial_node,
        hobot_usb_cam_launch,
        # cam_node,
        websocket_launch,
    ])