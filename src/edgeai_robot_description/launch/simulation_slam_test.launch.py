import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import xacro


def generate_launch_description():

    package_name = 'edgeai_robot_description'

    pkg_share = get_package_share_directory(package_name)
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Reuse existing robot model
    xacro_file = os.path.join(
        pkg_share,
        'urdf',
        'edgeai_robot_real.urdf.xacro'
    )

    # Independent 4 m x 4 m SLAM test world
    world_file = os.path.join(
        pkg_share,
        'worlds',
        'slam_test_4x4.sdf'
    )

    robot_description_config = xacro.process_file(xacro_file)

    robot_description = {
        'robot_description': robot_description_config.toxml()
    }

    # Start Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_ros_gz_sim,
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}'
        }.items()
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': True}
        ]
    )

    # Spawn robot in lower-left region
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'edgeai_robot_real',
            '-topic', 'robot_description',
            '-x', '-1.3',
            '-y', '-1.3',
            '-z', '0.01',
            '-Y', '0.0',
        ],
        output='screen'
    )

    # ROS 2 and Gazebo bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',

            # LiDAR
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',

            # RGB-D camera
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',

            # Simulation clock
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ]
    )

    # Static camera TF
    camera_sensor_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_sensor_tf',
        output='screen',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', 'camera_link',
            '--child-frame-id',
            'edgeai_robot_real/base_footprint/rgbd_camera',
        ]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        bridge,
        camera_sensor_tf,
    ])
