# launch/yolo_launch.py
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                '/home/heisenburg/venvs/numpy1241/bin/python3',
                '/home/heisenburg/agv/install/yolo/lib/yolo/yolo_publisher'
            ],
            output='screen',
            shell=False
        )
    ])
