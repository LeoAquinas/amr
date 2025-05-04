import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled

    package_name='amr'

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_controller': 'false'}.items()
    )

    twist_mux_params = os.path.join(get_package_share_directory(package_name), 'config', 'twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
    )

    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')

    world_path = "/home/heisenburg/agv/src/my_bot/worlds/obstacle.world"
    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'world': world_path, 'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
             )

    rviz_config_path = os.path.join(get_package_share_directory('amr'), 'config', 'my_bot.rviz')
    rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'amr'],
                        output='screen')

    # Run the diff drive spawner
    diff_drive_spawner = Node(package='controller_manager', executable='spawner',
                        arguments=['diff_cont'])

    # Run the joint broad spawner
    joint_broad_spawner = Node(package='controller_manager', executable='spawner',
                        arguments=['joint_broad'])
    
    # slam_params_file = "/home/heisenburg/agv/src/my_bot/config/mapper_params_online_async.yaml"
    # slam_toolbox = Node(
    #         package='slam_toolbox',
    #         executable='online_async_launch.py',
    #         name='slam_toolbox',
    #         parameters=[
    #             {'slam_params_file': slam_params_file,
    #              'use_sim_time': True}
    #             ],
    #         output='screen'
    # )


    # Launch them all!
    return LaunchDescription([
        rsp,
        twist_mux,
        gazebo,
        spawn_entity,
        rviz,
        diff_drive_spawner,
        joint_broad_spawner,
        # slam_toolbox
    ])