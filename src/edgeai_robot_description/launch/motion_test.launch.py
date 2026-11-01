from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # package_dir = get_package_share_directory('edgeai_robot_description')
    # rviz_config = os.path.join(package_dir, 'rviz', 'view.rviz')

    # static tf brodcaster：map -> odom
    
    # static_tf_node = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_map_to_realsense',
    #     arguments=[
    #         '0.49', '0.50', '0',        # x, y, z (in meters)
    #         '0', '0', '0',        # yaw, pitch, roll (in rads)
    #         'map',                # parent frame
    #         'odom'  # child frame
    #     ],
    #     output='screen'
    # )

    # static tf brodcaster：map -> base_link

    # static_tf_node = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_map_to_realsense',
    #     arguments=[
    #         '0.49', '0.50', '0',        # x, y, z (in meters)
    #         '0', '0', '0',        # yaw, pitch, roll (in rads)
    #         'map',                # parent frame
    #         'base_link'  # child frame
    #     ],
    #     output='screen'
    # )

    # static tf brodcaster：odom -> base_link
    
    # static_tf_node = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_odom_to_base_link',
    #     arguments=[
    #         '0', '0', '0',        # x, y, z (in meters)
    #         '0', '0', '0',        # yaw, pitch, roll (in rads)
    #         'odom',               # parent frame
    #         'base_link'  # child frame
    #     ],
    #     output='screen'
    # )

    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', rviz_config],
    #     output='screen',
    # )

    # realsense = ExecuteProcess(
    #     cmd=['pixi', 'run', 'realsense'],
    #     output='screen'
    # )

    # odometry = ExecuteProcess(
    #     cmd=['pixi', 'run', 'ros2', 'run', 'odometry', 'odometry'],
    #     output='screen'
    # )

    # phidgets = ExecuteProcess(
    #     cmd=['pixi', 'run', 'phidgets'],
    #     output='screen'
    # )

    # detection = ExecuteProcess(
    #     cmd=['pixi', 'run', 'ros2', 'run', 'detection', 'detection'],
    #     output='screen'
    # )

    motion_test = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '--rate',  '10', '/cmd_vel', 'geometry_msgs/msg/Twist', '{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.25}}'],
        output='screen'
    )

    return LaunchDescription([motion_test])