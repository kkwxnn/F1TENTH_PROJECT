import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

import xacro   
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument 
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    config_dir = os.path.join(get_package_share_directory('robot_bridge'), 'config')
    config_file = os.path.join(config_dir, 'mapper_params_online_sync.yaml')

    rviz_config_dir = os.path.join(get_package_share_directory('robot_bridge'), 'rviz')
    rviz_config_file = os.path.join(rviz_config_dir, 'mapping.rviz')

    pkg = get_package_share_directory('example_description')
    path_description = os.path.join(pkg,'robot','visual','robot.xacro')
    robot_desc_xml = xacro.process_file(path_description).toxml()

    parameters = [{'robot_description':robot_desc_xml}]
    #parameters.append({'frame_prefix':namespace+'/'})
    robot_state_publisher = Node(package='robot_state_publisher',
                                  executable='robot_state_publisher',
                                  output='screen',
                                  parameters=parameters
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher'
    )

    #=====================================================================#
    #==================== Generate Robot Bridge ==========================#
    #=====================================================================#

    RobotCommand_Node = Node(
                    package='robot_bridge',
                    executable='RobotCommand.py')
    
    ackerman_yaw_rate_odom_Node = Node(
                    package='robot_bridge',
                    executable='ackerman_yaw_rate_odom.py')
    
    mcu_bridge_Node = Node(
                    package='robot_bridge',
                    executable='mcu_bridge.py')
    
    #=====================================================================#
    #================= Generate Robot localization =======================#
    #=====================================================================#

    # pkg = get_package_share_directory('calibration_gen')
    # rviz_path = os.path.join(pkg,'config','_display.rviz')
    print(os.path.join(get_package_share_directory('robot_localization'), 'launch'), '/ekf.launch.py')

    ekf_node = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('robot_localization'), 'launch'), '/ekf.launch.py']),
             )


    #=====================================================================#
    #===================== Generate YDLidar ==============================#
    #=====================================================================#
    
    ydlidar_node = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ydlidar_ros2_driver'), 'launch'), '/ydlidar_launch.py']),
             )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='sync_slam_toolbox_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, config_file]),

        Node(
            package='rviz2',
            executable='rviz2',
            name='sync_slam_toolbox_node',
            output='screen',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}]),

        robot_state_publisher,
        joint_state_publisher,

        RobotCommand_Node,
        ackerman_yaw_rate_odom_Node,
        mcu_bridge_Node,
        ydlidar_node,
    ])

