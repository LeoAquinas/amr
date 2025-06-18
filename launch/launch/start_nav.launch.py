from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.actions import EmitEvent
from launch.events import Shutdown

def generate_launch_description():
    # Start ros2 bag record and keep a handle
    rosbag_process = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record', '--output',
            '/media/jetson/6252efc3-834a-466b-90d3-0028ea2e8da5/home/orin_nano/bag/lib22',
            '/camera/realsense2_camera/color/image_raw',
            '/camera/realsense2_camera/depth/color/points',
            '/camera/realsense2_camera/depth/image_rect_raw',
            '/camera/realsense2_camera/imu',
            '/camera/realsense2_camera/infra1/image_rect_raw',
            '/camera/realsense2_camera/infra2/image_rect_raw',
            '/clicked_point',
            '/cloud_ground',
            '/cloud_map',
            '/cloud_obstacles',
            '/cmd_vel',
            '/diagnostics',
            '/global_costmap/costmap',
            '/global_costmap/costmap_updates',
            '/global_path',
            '/global_path_nodes',
            '/global_pose',
            '/goal',
            '/goal_node',
            '/goal_pose',
            '/goal_reached',
            '/imu/data',
            '/info',
            '/initialpose',
            '/joint_states',
            '/labels',
            '/landmark_detection',
            '/landmark_detections',
            '/landmarks',
            '/local_costmap/costmap',
            '/local_costmap/costmap_updates',
            '/local_grid_empty',
            '/local_grid_ground',
            '/local_grid_obstacle',
            '/local_path',
            '/local_path_nodes',
            '/localization_pose',
            '/map',
            '/mapData',
            '/mapGraph',
            '/mapOdomCache',
            '/mapPath',
            '/map_updates',
            '/octomap_binary',
            '/octomap_full',
            '/octomap_ground',
            '/octomap_obstacles',
            '/octomap_occupied_space',
            '/odom_rf2o',
            '/odom_rgbd_image',
            '/odometry/filtered',
            '/parameter_events',
            '/plan',
            '/rgbd_image',
            '/rgbd_image/compressed',
            '/robot_description',
            '/rosout',
            '/rtabmap/republish_node_data',
            '/rtabmap_odom',
            '/rtabmap_odom_axis_swapped',
            '/scan',
            '/scan_filtered',
            '/set_pose',
            '/tf',
            '/tf_static',
            '/user_data_async',
            '/waypoints'
        ],
        output='screen',
        shell=False
    )

    # Python script, delayed 5 seconds
    python_script = TimerAction(
        period=7.0,
        actions=[
            ExecuteProcess(
                cmd=['python3', '/home/jetson/agv/src/amr/launch/simple_commander/lib2_turn.py'],
                # cmd=['python3', '/home/jetson/agv/src/amr/launch/simple_commander/lib3_door.py'],
                # cmd=['python3', '/home/jetson/agv/src/amr/launch/simple_commander/lib4_rack.py'],
                # cmd=['python3', '/home/jetson/agv/src/amr/launch/simple_commander/lib5_waypoint.py'],
                output='screen'
            )
        ]
    )

    # When the Python script ends, stop everything (including ros2 bag)
    on_script_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=python_script.actions[0],  # the ExecuteProcess inside TimerAction
            on_exit=[
                EmitEvent(event=Shutdown(reason='Python script finished'))
            ]
        )
    )

    return LaunchDescription([
        rosbag_process,
        python_script,
        on_script_exit,
    ])
