import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # Robot package
    robot_package = 'amr'
    description_n_rviz = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(robot_package),'launch','test_launch_sim.launch.py'
                )])
    )

    # Nav package
    navigation_package = 'launch_amr'
    navigation = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(navigation_package),'launch','demo_nav_simple_commander_without_voice.launch.py'
                )])
    )
    
    delayed_navigation = TimerAction(period=10.0, actions=[navigation])

    # Launch them all!
    return LaunchDescription([
        description_n_rviz,
        delayed_navigation
    ])