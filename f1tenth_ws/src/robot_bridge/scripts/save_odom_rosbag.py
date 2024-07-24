#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import csv

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import csv
import datetime
import pytz  # Import pytz for timezone support

class OdomRecorderNode(Node):

    def __init__(self):
        super().__init__('odom_recorder')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.file_name = 'Odom2_Obj_0.5.csv'
        self.csv_file = open(self.file_name, 'w')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['timestamp', 'x', 'y'])

        # self.subscription = self.create_subscription(
        #     Odometry,
        #     '/odom_tune',
        #     self.odom_tune_callback,
        #     10)
        # self.file_name_2 = 'Odom_tune_Circle_0.5_2.csv'
        # self.csv_file_2 = open(self.file_name_2, 'w')
        # self.csv_writer_2 = csv.writer(self.csv_file_2)
        # self.csv_writer_2.writerow(['timestamp', 'x', 'y'])
        
        # Define GMT+7 timezone
        self.timezone = pytz.timezone('Asia/Bangkok')  # Adjust to your specific GMT+7 timezone


    def ros_to_formatted_time(self, ros_time):
        # Convert ROS time (sec + nanosec) to datetime object
        epoch_start = datetime.datetime(1970, 1, 1, tzinfo=datetime.timezone.utc)
        ros_time_in_seconds = ros_time.sec + ros_time.nanosec * 1e-9
        dt_object_utc = epoch_start + datetime.timedelta(seconds=ros_time_in_seconds)
        
        # Convert UTC datetime to GMT+7
        dt_object_gmt7 = dt_object_utc.astimezone(self.timezone)
        
        # Format datetime object as string
        formatted_time = dt_object_gmt7.strftime('%Y-%m-%d %H:%M:%S.%f')
        return formatted_time

    def odom_callback(self, msg):
        # Convert ROS time to formatted time string in GMT+7
        recorded_time = self.ros_to_formatted_time(msg.header.stamp)
        self.get_logger().info(f"Position: ({msg.pose.pose.position.x}, {msg.pose.pose.position.y})")
        self.csv_writer.writerow([recorded_time,
                                  msg.pose.pose.position.x,
                                  msg.pose.pose.position.y])
        
    # def odom_tune_callback(self, msg):
    #     # Convert ROS time to formatted time string in GMT+7
    #     recorded_time = self.ros_to_formatted_time(msg.header.stamp)
    #     self.get_logger().info(f"Position: ({msg.pose.pose.position.x}, {msg.pose.pose.position.y})")
    #     self.csv_writer_2.writerow([recorded_time,
    #                               msg.pose.pose.position.x,
    #                               msg.pose.pose.position.y])

def main(args=None):
    rclpy.init(args=args)
    odom_recorder = OdomRecorderNode()
    rclpy.spin(odom_recorder)
    odom_recorder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


