#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion, TransformStamped, PoseWithCovarianceStamped
import numpy as np
from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage
import tf_transformations
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from robot_navigator import BasicNavigator, NavigationResult 
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class ViaPointGenerateNode(Node):
    def __init__(self):
        super().__init__('via_point_generate_node')
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.current_pose_callback,
            10)
        self.create_timer(0.033, self.timer_callback)
        self.lasttimestamp = self.get_clock().now()

        self.via_point = [[0.25,np.pi / 4.0],
                     [0.0,0.0]]
        
        self.time_stamp = [1.0,
                           4.0,]
        
        self.index = 0
        self.t = 0.0
        self.cmd_vel = [0.0, 0.0]

        self.current_pose_x = 0.0
        self.current_pose_y = 0.0
        self.current_pose_yaw = 0.0

        self.goal_indx = 0
        self.current_goal_pose = [[5.35, -0.37, -np.deg2rad(90)], [2.27, -0.9, np.deg2rad(90)]]

        self.goal_pose_x = 0.0
        self.goal_pose_y = 0.0
        self.goal_pose_yaw = 0.0

        self.error_threshold = [1.5, 4.0]

        # Declare and acquire `target_frame` parameter
        self.target_frame = self.declare_parameter(
          'target_frame', 'base_footprint').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
 
        # Launch the ROS 2 Navigation Stack
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()

    def current_pose_callback(self, msg):
        self.current_pose_x = msg.pose.pose.position.x
        self.current_pose_y = msg.pose.pose.position.y
        orientation_list = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        _, _, self.current_pose_yaw = tf_transformations.euler_from_quaternion(orientation_list)

    def send_goal(self, goal):
        # Set the robot's goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = goal[0]
        goal_pose.pose.position.y = goal[1]
        goal_pose.pose.position.z = 0.0
        orientation_list = tf_transformations.quaternion_from_euler(0.0, 0.0, goal[2])
        goal_pose.pose.orientation.x = orientation_list[0]
        goal_pose.pose.orientation.y = orientation_list[1]
        goal_pose.pose.orientation.z = orientation_list[2]
        goal_pose.pose.orientation.w = orientation_list[3]

        # Go to the goal pose
        self.navigator.goToPose(goal_pose)

    def timer_callback(self):
        # calculate dt
        currenttimestamp = self.get_clock().now()
        dt = (currenttimestamp - self.lasttimestamp).to_msg().nanosec * 1.0e-9
        self.lasttimestamp = currenttimestamp
        self.t += dt

        # from_frame_rel = self.target_frame
        # to_frame_rel = 'map'
        # try:
        #     t = self.tf_buffer.lookup_transform(
        #         to_frame_rel,
        #         from_frame_rel,
        #         rclpy.time.Time())
        # except TransformException as ex:
        #     self.get_logger().info(
        #         f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
        #     return

        error_pose_x = abs(self.goal_pose_x - self.current_pose_x)
        error_pose_y = abs(self.goal_pose_y - self.current_pose_y)
        print(error_pose_x, error_pose_y)

        if error_pose_x < self.error_threshold[0] and error_pose_y < self.error_threshold[1]:
            print("reach goal")
            self.send_goal(self.current_goal_pose[self.goal_indx])
            self.goal_pose_x = self.current_goal_pose[self.goal_indx][0]
            self.goal_pose_y = self.current_goal_pose[self.goal_indx][1]
            
            if self.goal_indx == 0:
                self.goal_indx = 1
            else:
                self.goal_indx = 0 

            

def main(args=None):
    rclpy.init(args=args)
    node = ViaPointGenerateNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
