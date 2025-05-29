import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

            Node(
                package='realsense2_camera',
                executable='realsense2_camera_node',
                name='realsense2_camera',
                output='screen',
                parameters=[{
                    'camera_name' : 'camera',
                    'camera_namespace' : 'camera',
                    'pointcloud.enable' : True,
                    'enable_gyro' : True,
                    'enable_accel' : True, 
                    'enable_color' : True,
                    'unite_imu_method': 2,

                    # 'enable_depth' : True,
                    # 'align_depth' : True,
                    'enable_sync' : True,

                    'rgb_camera.power_line_frequency' : 1,

                    # Need to change frame-rate and res in order to not freeze up RGB publisher
                    'rgb_camera.color_profile' : '424, 240, 15',
                    'depth_module.depth_profile' : '424, 240, 15',
                    # 'rgb_camera.color_profile' : '212, 120, 15',
                    # 'depth_module.depth_profile' : '212, 120, 15',
                    
                    'enable_infra' : True,
                    'enable_infra1' : True,
                    'enable_infra2' : True,

                    'depth_module.infra_profile' : '424, 240, 15'
                    }],
            ),
    ])