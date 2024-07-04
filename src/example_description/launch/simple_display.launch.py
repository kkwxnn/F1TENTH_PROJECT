#!/usr/bin/env python3

"""
This program is free software: you can redistribute it and/or modify it 
under the terms of the GNU General Public License as published by the Free Software Foundation, 
either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program. 
If not, see <https://www.gnu.org/licenses/>.

created by Thanacha Choopojcharoen at CoXsys Robotics (2022)
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os
import xacro   
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler 
from launch.event_handlers import OnProcessExit
    
def generate_launch_description():
    
    pkg = get_package_share_directory('example_description')
    rviz_path = os.path.join(pkg,'config','_display.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_path],
        output='screen')
    
    path_description = os.path.join(pkg,'robot','visual','robot.xacro')
    robot_desc_xml = xacro.process_file(path_description).toxml()
    #robot_desc_xml = xacro.process_file(path_description,mappings={'robot_name': namespace}).toxml()

    # urdf_file_name = 'robot.urdf'
    # urdf = os.path.join(pkg,'robot','visual',urdf_file_name)
    # with open(urdf, 'r') as infp:
    #     robot_desc = infp.read()

    
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


    # launch_description = LaunchDescription()
    
    # launch_description.add_action(rviz)
    # launch_description.add_action(robot_state_publisher)
    # # launch_description.add_action(joint_state_publisher_gui)
    # launch_description.add_action(joint_state_broadcaster_spawner)
    # launch_description.add_action(velocity_controllers)
    
    # return launch_description

    # ***** RETURN LAUNCH DESCRIPTION ***** #
    return LaunchDescription([
        
        rviz, 
        robot_state_publisher,
        joint_state_publisher,

    ])