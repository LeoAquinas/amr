import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch.event_handlers import OnProcessStart
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node


# DONE:
'''
Moved lidar startup
Moved lidar filter
Moved camera startup
Moved rf2o
Moved rtabmap
'''

#TODO:
'''
Uncomment rf2o
Include kalman filter result for odom topic
include kalman filter initialization
'''


def generate_launch_description():

    # LiDAR Startupsrc/launch
    lidar_package_name = 'sllidar_ros2'

    lidar = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(lidar_package_name),'launch','view_sllidar_a1_launch.py'
                )])
    )

    # LiDAR filter
    lidar_filter = Node(
                    package="laser_filters",
                    executable="scan_to_scan_filter_chain",
                    parameters=[
                        PathJoinSubstitution([
                            get_package_share_directory("laser_filters"),
                            "examples", "angular_filter_example.yaml",
                        ])],
                )

    #Rf2o
    rf2o = Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            output='screen',
            parameters=[{
                'laser_scan_topic' : '/scan_filtered',
                'odom_topic' : '/odom_rf2o',
                'publish_tf' : True,
                'base_frame_id' : 'base_link',
                'odom_frame_id' : 'odom',
                'init_pose_from_topic' : '',
                'freq' : 20.0}],
        )

    # Realsense startup
    realsense_package_name = 'realsense2_camera'

    realsense = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(realsense_package_name),'launch','camera_launch.launch.py'
                )])
    )

    # RTAB-Map startup
    rtab_package_name = 'rtabmap_launch'
    # odom_topic = "/odom_rf2o"
    odom_topic = "/odometry/filtered"

    rtabmap = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(rtab_package_name),'launch','rtabmap.launch.py'
                )]), launch_arguments={'subscribe_rgbd': 'true', 'rtabmap_args': '--delete_db_on_start', 'odom_topic': odom_topic}.items()
    )

    # RTAB-Map startup
    rtab_package_name = 'rtabmap_launch'
    odom_topic = "/odom_rf2o"

    rtabmap = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(rtab_package_name),'launch','rtabmap.launch.py'
                )]), launch_arguments={'subscribe_rgbd': 'false', 'rtabmap_args': '--delete_db_on_start', 'odom_topic': odom_topic}.items()
    )

    # ORBSLAM3 startup
    vocabulary_file = '/home/jetson/agv/src/vslam/orbslam3_ros2/vocabulary/ORBvoc.txt'
    config_file = '/home/jetson/agv/src/vslam/orbslam3_ros2/config/stereo/RealSense_D435i.yaml'
    rectify = 'false'

    orbslam3 = ExecuteProcess(
            cmd=[
                'ros2', 'run', 'orbslam3', 'stereo',
                vocabulary_file,
                config_file,
                rectify
            ],
            output='screen'
        )
    

    yaml_file_path = "/home/jetson/agv/src/others/robot_localization/params/ukf.yaml"
    ukf = Node(
            package='robot_localization',
            executable='ukf_node',
            name='ukf_filter_node',
            output='screen',
            parameters=[yaml_file_path, {'use_sim_time': False}],
           )
    
    # Launch them all!
    return LaunchDescription([
        lidar,
        lidar_filter,
        rf2o,
        realsense,
        orbslam3,
        # ukf,
        rtabmap
    ])