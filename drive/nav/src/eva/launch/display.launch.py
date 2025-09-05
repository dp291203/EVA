from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    # Path to your URDF file
    urdf_file_path = os.path.join(
        get_package_share_directory('eva'),
        'urdf',
        'eva.urdf'
    )

    # Read the URDF contents
    with open(urdf_file_path, 'r') as infp:
        robot_description_content = infp.read()

    return LaunchDescription([
        # Joint State Publisher GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content}]
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            # No config file, launch RViz with default settings
            # arguments=['-d', '/home/divya/ros2_ws/src/eva/rviz/default.rviz']
            arguments=[]
        ),
        
        #Node(
         #   package='eva',
         #  executable='odom_to_tf_broadcaster',
         #   name='odom_to_tf_broadcaster',
         #   output='screen',
        #)
    ])
