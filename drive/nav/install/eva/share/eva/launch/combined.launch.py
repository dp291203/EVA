# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
# from ament_index_python.packages import get_package_share_directory

# import os

# def generate_launch_description():
#     # Path to your URDF file
#     urdf_file_path = os.path.join(
#         get_package_share_directory('eva'),
#         'urdf',
#         'eva.urdf'
#     )

#     # Read the URDF contents
#     with open(urdf_file_path, 'r') as infp:
#         robot_description_content = infp.read()

#     # Lidar launch configurations
#     channel_type = LaunchConfiguration('channel_type', default='serial')
#     serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
#     serial_baudrate = LaunchConfiguration('serial_baudrate', default='256000')
#     frame_id = LaunchConfiguration('frame_id', default='lidar')  # match your URDF link name
#     inverted = LaunchConfiguration('inverted', default='false')
#     angle_compensate = LaunchConfiguration('angle_compensate', default='true')

#     return LaunchDescription([
#         # Declare lidar arguments
#         DeclareLaunchArgument('channel_type', default_value=channel_type),
#         DeclareLaunchArgument('serial_port', default_value=serial_port),
#         DeclareLaunchArgument('serial_baudrate', default_value=serial_baudrate),
#         DeclareLaunchArgument('frame_id', default_value=frame_id),
#         DeclareLaunchArgument('inverted', default_value=inverted),
#         DeclareLaunchArgument('angle_compensate', default_value=angle_compensate),

#         # Joint State Publisher GUI
#         Node(
#             package='joint_state_publisher_gui',
#             executable='joint_state_publisher_gui',
#             name='joint_state_publisher_gui',
#             output='screen',
#         ),

#         # Robot State Publisher
#         Node(
#             package='robot_state_publisher',
#             executable='robot_state_publisher',
#             name='robot_state_publisher',
#             output='screen',
#             parameters=[{'robot_description': robot_description_content}]
#         ),

#         # RViz
#         Node(
#             package='rviz2',
#             executable='rviz2',
#             name='rviz2',
#             output='screen',
#             arguments=[]
#         ),

#         # SL-LiDAR Node
#         Node(
#             package='sllidar_ros2',
#             executable='sllidar_node',
#             name='sllidar_node',
#             parameters=[{
#                 'channel_type': channel_type,
#                 'serial_port': serial_port,
#                 'serial_baudrate': serial_baudrate,
#                 'frame_id': frame_id,
#                 'inverted': inverted,
#                 'angle_compensate': angle_compensate
#             }],
#             output='screen'
#         ),
#     ])
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

    # Lidar launch configurations
    channel_type = LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='256000')
    frame_id = LaunchConfiguration('frame_id', default='lidar')  # match your URDF link name
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')

    return LaunchDescription([
        # Declare lidar arguments
        DeclareLaunchArgument('channel_type', default_value=channel_type),
        DeclareLaunchArgument('serial_port', default_value=serial_port),
        DeclareLaunchArgument('serial_baudrate', default_value=serial_baudrate),
        DeclareLaunchArgument('frame_id', default_value=frame_id),
        DeclareLaunchArgument('inverted', default_value=inverted),
        DeclareLaunchArgument('angle_compensate', default_value=angle_compensate),

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
            arguments=[]
        )

        # # SL-LiDAR Node  
        # Node(
        #     package='sllidar_ros2',
        #     executable='sllidar_node',
        #     name='sllidar_node',
        #     parameters=[{
        #         'channel_type': channel_type,
        #         'serial_port': serial_port,
        #         'serial_baudrate': serial_baudrate,
        #         'frame_id': frame_id,
        #         'inverted': inverted,
        #         'angle_compensate': angle_compensate
        #     }],
        #     output='screen'
        # ),

        # # Odom Publisher Node
        # Node(
        #     package='eva',
        #     executable='odom_publisher',
        #     name='odom_publisher',
        #     output='screen',
        # ),
    ])
