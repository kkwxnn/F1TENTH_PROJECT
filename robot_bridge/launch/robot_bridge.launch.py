#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os
import xacro   
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument 
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
    
def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_dir = os.path.join(get_package_share_directory(
        'robot_bridge'), 'maps')
    map_file = LaunchConfiguration('map', default=os.path.join(
        map_dir, 'FIBO_floor5_AMCL.yaml'))

    param_dir = os.path.join(get_package_share_directory(
        'robot_bridge'), 'config')
    param_file = LaunchConfiguration(
        'params', default=os.path.join(param_dir, 'navigation_param.yaml'))

    nav2_launch_file_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch')

    #=====================================================================#
    #======================== Generate RVIZ ==============================#
    #=====================================================================#
    pkg = get_package_share_directory('example_description')
    rviz_path = os.path.join(pkg,'config','robot_bringup.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_path],
        output='screen')
    
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

    # pkg = get_package_share_directory('robot_bridge')
    # rviz_path = os.path.join(pkg,'config','_display.rviz')

    # DiffDriveRobot = Node(
    #                 package='robot_bridge',
    #                 executable='diff_drive_robot.py')
    
    # CommandOdom = Node(
    #                 package='robot_bridge',
    #                 executable='cmd_odom.py')

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
    #====================== Generate IMU Node ============================#
    #=====================================================================#

    # pkg = get_package_share_directory('calibration_gen')
    # rviz_path = os.path.join(pkg,'config','_display.rviz')

    # imuread_node = Node(
                    # package='calibration_gen',
                    # executable='imuread_node.py')
    
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

    # ackerman_node = Node(
    #                 package='ackerman_odometry',
    #                 executable='odometry_calculation.py')

    #=====================================================================#
    #===================== Generate YDLidar ==============================#
    #=====================================================================#
    
    ydlidar_node = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ydlidar_ros2_driver'), 'launch'), '/ydlidar_launch.py']),
             )
        

    # ***** RETURN LAUNCH DESCRIPTION ***** #
    return LaunchDescription([

        DeclareLaunchArgument(
            'map',
            default_value=map_file,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params',
            default_value=param_file,
            description='Full path to param file to load'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_file,
                'use_sim_time': use_sim_time,
                'params_file': param_file}.items(),
        ),
        
        rviz, 

        robot_state_publisher,
        joint_state_publisher,

        # DiffDriveRobot,
        # CommandOdom,
        # imuread_node,
        # ekf_node,
        # ackerman_node,

        RobotCommand_Node,
        ackerman_yaw_rate_odom_Node,
        mcu_bridge_Node,
        # ydlidar_node,
    ])
