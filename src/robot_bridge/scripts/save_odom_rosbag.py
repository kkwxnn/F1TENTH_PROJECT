#!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Odometry
# import csv

# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Odometry
# import csv
# import datetime
# import pytz  # Import pytz for timezone support

# class OdomRecorderNode(Node):

#     def __init__(self):
#         super().__init__('odom_recorder')
#         self.subscription = self.create_subscription(
#             Odometry,
#             '/odom',
#             self.odom_callback,
#             10)
#         self.file_name = 'AMCL_Obj_0.5.csv'
#         self.csv_file = open(self.file_name, 'w')
#         self.csv_writer = csv.writer(self.csv_file)
#         self.csv_writer.writerow(['timestamp', 'x', 'y'])

#         # self.subscription = self.create_subscription(
#         #     Odometry,
#         #     '/odom_tune',
#         #     self.odom_tune_callback,
#         #     10)
#         # self.file_name_2 = 'Odom_tune_Circle_0.5_2.csv'
#         # self.csv_file_2 = open(self.file_name_2, 'w')
#         # self.csv_writer_2 = csv.writer(self.csv_file_2)
#         # self.csv_writer_2.writerow(['timestamp', 'x', 'y'])
        
#         # Define GMT+7 timezone
#         self.timezone = pytz.timezone('Asia/Bangkok')  # Adjust to your specific GMT+7 timezone


#     def ros_to_formatted_time(self, ros_time):
#         # Convert ROS time (sec + nanosec) to datetime object
#         epoch_start = datetime.datetime(1970, 1, 1, tzinfo=datetime.timezone.utc)
#         ros_time_in_seconds = ros_time.sec + ros_time.nanosec * 1e-9
#         dt_object_utc = epoch_start + datetime.timedelta(seconds=ros_time_in_seconds)
        
#         # Convert UTC datetime to GMT+7
#         dt_object_gmt7 = dt_object_utc.astimezone(self.timezone)
        
#         # Format datetime object as string
#         formatted_time = dt_object_gmt7.strftime('%Y-%m-%d %H:%M:%S.%f')
#         return formatted_time

#     def odom_callback(self, msg):
#         # Convert ROS time to formatted time string in GMT+7
#         recorded_time = self.ros_to_formatted_time(msg.header.stamp)
#         self.get_logger().info(f"Position: ({msg.pose.pose.position.x}, {msg.pose.pose.position.y})")
#         self.csv_writer.writerow([recorded_time,
#                                   msg.pose.pose.position.x,
#                                   msg.pose.pose.position.y])
        
#     # def odom_tune_callback(self, msg):
#     #     # Convert ROS time to formatted time string in GMT+7
#     #     recorded_time = self.ros_to_formatted_time(msg.header.stamp)
#     #     self.get_logger().info(f"Position: ({msg.pose.pose.position.x}, {msg.pose.pose.position.y})")
#     #     self.csv_writer_2.writerow([recorded_time,
#     #                               msg.pose.pose.position.x,
#     #                               msg.pose.pose.position.y])

# def main(args=None):
#     rclpy.init(args=args)
#     odom_recorder = OdomRecorderNode()
#     rclpy.spin(odom_recorder)
#     odom_recorder.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_msgs.msg import TFMessage
import csv
import datetime
import pytz

class OdomRecorderNode(Node):

    def __init__(self):
        super().__init__('odom_recorder')
        
        # Subscription to /odom
        self.subscription_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        
        # Subscription to /amcl_pose
        self.subscription_amcl = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_callback,
            10)
        
        # Subscription to /tf
        self.subscription_tf = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            10)
        
        # File setup
        self.file_name = 'AMCL_Circle_0.5.csv'
        self.csv_file = open(self.file_name, 'w')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['timestamp', 'odom_x', 'odom_y', 'amcl_x', 'amcl_y', 'map_odom_x', 'map_odom_y', 'odom_base_footprint_x', 'odom_base_footprint_y'])
        
        # Define GMT+7 timezone
        self.timezone = pytz.timezone('Asia/Bangkok')  # Adjust to your specific GMT+7 timezone
        
        # Initialize dictionaries to store the latest positions
        self.latest_odom = {'timestamp': '', 'x': '', 'y': ''}
        self.latest_amcl = {'timestamp': '', 'x': '', 'y': ''}
        self.latest_tf_map_odom = {'timestamp': '', 'x': '', 'y': ''}
        self.latest_tf_odom_base_footprint = {'timestamp': '', 'x': '', 'y': ''}

    def ros_to_formatted_time(self, ros_time):
        epoch_start = datetime.datetime(1970, 1, 1, tzinfo=datetime.timezone.utc)
        ros_time_in_seconds = ros_time.sec + ros_time.nanosec * 1e-9
        dt_object_utc = epoch_start + datetime.timedelta(seconds=ros_time_in_seconds)
        dt_object_gmt7 = dt_object_utc.astimezone(self.timezone)
        formatted_time = dt_object_gmt7.strftime('%Y-%m-%d %H:%M:%S.%f')
        return formatted_time

    def write_to_csv(self):
        # Ensure at least one timestamp is available
        timestamp = self.latest_odom['timestamp'] or self.latest_amcl['timestamp'] or self.latest_tf_map_odom['timestamp'] or self.latest_tf_odom_base_footprint['timestamp']
        
        self.csv_writer.writerow([
            timestamp,
            self.latest_odom['x'], self.latest_odom['y'],
            self.latest_amcl['x'], self.latest_amcl['y'],
            self.latest_tf_map_odom['x'], self.latest_tf_map_odom['y'],
            self.latest_tf_odom_base_footprint['x'], self.latest_tf_odom_base_footprint['y']
        ])

    def odom_callback(self, msg):
        recorded_time = self.ros_to_formatted_time(msg.header.stamp)
        self.latest_odom = {
            'timestamp': recorded_time,
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y
        }
        self.get_logger().info(f"Odometry Position: ({msg.pose.pose.position.x}, {msg.pose.pose.position.y})")
        self.write_to_csv()
        
    def amcl_pose_callback(self, msg):
        recorded_time = self.ros_to_formatted_time(msg.header.stamp)
        self.latest_amcl = {
            'timestamp': recorded_time,
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y
        }
        self.get_logger().info(f"AMCL Position: ({msg.pose.pose.position.x}, {msg.pose.pose.position.y})")
        self.write_to_csv()
        
    def tf_callback(self, msg):
        for transform in msg.transforms:
            if transform.child_frame_id == 'odom' and transform.header.frame_id == 'map':  # map -> odom
                recorded_time = self.ros_to_formatted_time(transform.header.stamp)
                self.latest_tf_map_odom = {
                    'timestamp': recorded_time,
                    'x': transform.transform.translation.x,
                    'y': transform.transform.translation.y
                }
                self.get_logger().info(f"TF map -> odom Position: ({transform.transform.translation.x}, {transform.transform.translation.y})")
            elif transform.child_frame_id == 'base_footprint' and transform.header.frame_id == 'odom':  # odom -> base_footprint
                recorded_time = self.ros_to_formatted_time(transform.header.stamp)
                self.latest_tf_odom_base_footprint = {
                    'timestamp': recorded_time,
                    'x': transform.transform.translation.x,
                    'y': transform.transform.translation.y
                }
                self.get_logger().info(f"TF odom -> base_footprint Position: ({transform.transform.translation.x}, {transform.transform.translation.y})")
                
            self.write_to_csv()

def main(args=None):
    rclpy.init(args=args)
    odom_recorder = OdomRecorderNode()
    rclpy.spin(odom_recorder)
    odom_recorder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



