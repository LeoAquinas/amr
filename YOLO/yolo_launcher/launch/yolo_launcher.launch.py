# launch/yolo_launch.py
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                '/home/jetson/venvs/numpy1241/bin/python3',
                '/home/jetson/agv/install/yolo/lib/yolo/yolo_publisher'    # Change dir path
            ],
            output='screen',
            shell=False
        ),

        # Another node running with the system Python
        # ExecuteProcess(
        #     cmd=[
        #         '/usr/bin/python3',
        #         '/home/heisenburg/Desktop/test_voice.py'
        #     ],
        #     output='screen',
        #     shell=False
        # ),
    ])
