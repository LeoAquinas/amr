import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch.event_handlers import OnProcessStart, OnProcessExit
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
                )]), launch_arguments={'serial_port': '/dev/rplidar'
                                        }.items()
    )

    # LiDAR filter
    lidar_filter = Node(
                    package="laser_filters",
                    executable="scan_to_scan_filter_chain",
                    parameters=[
                        PathJoinSubstitution([
                            get_package_share_directory("laser_filters"),
                            "examples", "angular_filter_example.yaml",
                        ])]
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
    # odom_topic = "/rtabmap_odom"
    odom_topic = "/odometry/filtered"

    rtabmap = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(rtab_package_name),'launch','rtabmap.launch.py'
                )]), launch_arguments={'database_path': '/home/jetson/agv/src/amr/launch/map/rtabmap_plc.db',
                                       'use_sim_time': 'false',
                                       'rtabmap_viz': 'false',
                                       'localization': 'false',
                                       'subscribe_rgbd': 'true',
                                       'rtabmap_args': '--delete_db_on_start',
                                       'rgbd_sync': 'true',
                                       'approx_rgbd_sync': 'true',
                                       'compressed': 'false',

                                       'subscribe_odom_info': 'true',
                                       'visual_odometry': 'true',
                                       'icp_odometry': 'false',
                                       'vo_frame_id': 'odom',
                                       'odom_topic': odom_topic,
                                       'publish_tf_odom': 'false',
                                       'odom_sensor_sync': 'true',

                                       'imu_topic': '/imu/data',
                                       'wait_imu_to_init': 'true',
                                    #    'always_check_imu_tf': 'true',
                                       'qos_imu': '2',

                                        'subscribe_scan': 'true',
                                        'subscribe_scan_cloud': 'false',

                                       'scan_topic': '/scan_filtered',
                                       'rgb_topic': '/camera/realsense2_camera/color/image_raw',
                                       'depth_topic': '/camera/realsense2_camera/depth/image_rect_raw',
                                       'camera_info_topic': '/camera/realsense2_camera/color/camera_info',
                                       
                                       'stereo': 'true',
                                       'stereo_namespace': '',
                                       'left_image_topic': '/camera/realsense2_camera/infra1/image_rect_raw',
                                       'right_image_topic': '/camera/realsense2_camera/infra2/image_rect_raw',
                                       'left_camera_info_topic': '/camera/realsense2_camera/infra1/camera_info',
                                       'right_camera_info_topic': '/camera/realsense2_camera/infra2/camera_info',

                                        # Custom params
                                        'grid_raytracing':'true', # Fill empty  space
                                        'grid_3d':'false', # Use 2D occupancy
                                        'min_cluster_size':'10',


                                        'Grid/RangeMax':'3',
                                        'Grid/NormalsSegmentation':'false', # Use passthrough filter to detect obstacles
                                        # Create occupancy grid from selected sensor: 0=laser scan, 1=depth image(s) or 2=both laser scan and depth image(s).
                                        ## CHANGE DEPENDING ON NEED
                                        'Grid/Sensor':'1', # Use both laser scan and camera for obstacle detection in global map
                                        'Grid/MaxGroundHeight':'0.02', # All points above 5 cm are obstacles
                                        'Grid/MaxObstacleHeight':'1.8',  # All points over 1 meter are ignored

                                       }.items()
    )

    # Compute quaternion of the IMU
    quaternion = Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            output='screen',
            parameters=[{'use_mag': False, 
                         'world_frame':'enu', 
                         'publish_tf':False}],
            remappings=[('imu/data_raw', '/camera/realsense2_camera/imu')]
            )
    
    # UKF
    yaml_file_path = "/home/jetson/agv/src/others/robot_localization/params/ukf.yaml"
    ukf = Node(
            package='robot_localization',
            executable='ukf_node',
            name='ukf_filter_node',
            output='screen',
            parameters=[yaml_file_path, {'use_sim_time': False}],
           )
    
    ''' TODO:
        Technically not used so can remove
    # ORBSLAM3 stereo startup
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

    #LiDAR rotator
    lidar_rotator = Node(
                    package="lidar_rotator",
                    executable="lidar_rotator"
                )
    
    # Add delays for sequential execution
    delayed_realsense = TimerAction(period=2.0, actions=[realsense])
    delayed_orbslam3 = TimerAction(period=3.0, actions=[orbslam3])
    delayed_ukf = TimerAction(period=13.0, actions=[ukf])
    delayed_rtabmap = TimerAction(period=14.0, actions=[rtabmap])

    '''
    delayed_rtabmap = TimerAction(period=7.0, actions=[rtabmap])

    # Launch them all!
    return LaunchDescription([
        lidar,
        lidar_filter,
        # # lidar_rotator,
        rf2o,
        realsense,
        # # orbslam3,
        ukf,
        quaternion,
        # rtabmap,


        # delayed_realsense,
        # delayed_orbslam3,
        # delayed_ukf,
        delayed_rtabmap
    ])