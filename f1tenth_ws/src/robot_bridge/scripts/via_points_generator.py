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
        
        self.time_stamp = [1.0,
                           4.0,]
        
        self.index = 0
        self.t = 0.0
        self.cmd_vel = [0.0, 0.0]

        self.current_pose_x = 0.0
        self.current_pose_y = 0.0
        self.current_pose_yaw = 0.0

        self.goal_indx = 0
        # self.current_goal_pose = [[5.0, 0.0, np.deg2rad(0)], [5.85, 5.5, np.deg2rad(180)], [0.42, 4.23, np.deg2rad(-90)], [0.42, 0.67, np.deg2rad(-45)]]
        #self.current_goal_pose = [[3.38, 1.57, np.deg2rad(90)], [5.98, 5.46, np.deg2rad(0)], [2.31,5.59, np.deg2rad(0)], [5.15, 0.06, np.deg2rad(0)], [1.17, 0.26, np.deg2rad(180)]]
        self.current_goal_pose = [[3.636773109436035,-0.06060022488236427, np.deg2rad(0)], 
                                  [7.641051292419434, -2.091191530227661, np.deg2rad(-130)],
                                  [6.6992716789245605, -4.1122517585754395, np.deg2rad(-130)],
                                  [3.304191827774048,-5.515702724456787, np.deg2rad(-180)],
                                  [0.15268611907958984,-5.585211277008057, np.deg2rad(-180)],
                                  [-0.7673523426055908,-3.6333212852478027, np.deg2rad(90)],
                                  [-0.8614640235900879, -1.3433672189712524, np.deg2rad(90)]]
        
        # [[7.608312606811523,3.8289072513580322, np.deg2rad(0)], 
        #                           [1.6564301252365112,0.3198237419128418, np.deg2rad(0)],
        #                           [1.6392544507980347,5.711426734924316, np.deg2rad(0)],
        #                           [3.7355315685272217,4.159043788909912, np.deg2rad(0)],
        #                           [1.1766549348831177,2.9632551670074463, np.deg2rad(0)],
        #                           [4.93858003616333,0.1032109260559082, np.deg2rad(0)]]
        
        # self.current_goal_pose = [[1.8990306854248047, -1.1380865573883057, np.deg2rad(-90)],
        #                           [3.5868537425994873, -2.704939365386963, np.deg2rad(0)],
        #                         [6.806461334228516,-3.3339898586273193, np.deg2rad(-35)],
        #                         [5.593564510345459, -5.437044620513916, np.deg2rad(180)],
        #                         [1.9999635219573975,-4.028874397277832, np.deg2rad(90)],
        #                         [0.24129532277584076,-2.776916742324829, np.deg2rad(180)],
        #                         [-0.8163034915924072, -1.4947936534881592, np.deg2rad(90)],]

        self.goal_pose_x = None
        self.goal_pose_y = None
        self.goal_pose_yaw = 0.0

        #5 1 2 3 1 6 2

        #self.error_threshold = [2.5, 2.5]
        self.error_threshold = [2.0, 2.0]

        # Declare and acquire `target_frame` parameter
        # self.target_frame = self.declare_parameter(
        #   'target_frame', 'base_footprint').get_parameter_value().string_value

        # self.tf_buffer = Buffer()
        # self.tf_listener = TransformListener(self.tf_buffer, self)
 
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
        # currenttimestamp = self.get_clock().now()
        # dt = (currenttimestamp - self.lasttimestamp).to_msg().nanosec * 1.0e-9
        # self.lasttimestamp = currenttimestamp
        # self.t += dt

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

        if self.goal_pose_x == None:
            self.send_goal(self.current_goal_pose[self.goal_indx])
            self.goal_pose_x = self.current_goal_pose[self.goal_indx][0]
            self.goal_pose_y = self.current_goal_pose[self.goal_indx][1]
            self.goal_indx = 1
        else:

            error_pose_x = abs(self.goal_pose_x - self.current_pose_x)
            error_pose_y = abs(self.goal_pose_y - self.current_pose_y)
            # print(self.goal_pose_x, self.goal_pose_y, self.goal_indx)
        
            

            if error_pose_x < self.error_threshold[0] and error_pose_y < self.error_threshold[1]:
                print("reach goal")
                self.send_goal(self.current_goal_pose[self.goal_indx])
                self.goal_pose_x = self.current_goal_pose[self.goal_indx][0]
                self.goal_pose_y = self.current_goal_pose[self.goal_indx][1]
                
                if self.goal_indx != len(self.current_goal_pose) - 1:
                    self.goal_indx += 1
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
