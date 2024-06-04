#!/usr/bin/python3

from robot_bridge.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
import csv
import os
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu

class CSVWritterNode(Node):
    def __init__(self):
        super().__init__('csv_writer_node')
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.create_subscription(Imu, "/imu", self.imu_callback, 10)
        self.create_subscription(Float64MultiArray, "/motor_speed", self.motor_speed_callback, 10)
        self.create_timer(0.033, self.timer_callback)
        self.odom_x = None
        self.odom_y = None
        self.cmd_vx = None
        self.cmd_w = None
        self.imu_yaw = None

        self.wheel_radius = 0.033
        self.wheel_speed = 0.0
        self.linear_speed = 0.0

        self.wheel2motor_ratio = (27/68) * (15/39)

        # name of csv file
        filename = "save.csv"
    
        self.path = os.path.join('/home/jetson/f1tenth_ws', filename)

        # field names
        fields = ['odom_x', 'odom_y', 'linear_speed', 'cmd_vx', 'cmd_w', 'imu_yaw']

        # writing to csv file
        with open(self.path, 'w') as csvfile:
            # creating a csv writer object
            csvwriter = csv.writer(csvfile)
            # writing the fields
            csvwriter.writerow(fields)

    def timer_callback(self):
            
        if self.cmd_vx == 0.0 and self.cmd_w == 0.0:
            print("Stop Record: Robot Stop")
        else:
            print("Wheel Vel: ", self.cmd_vx, self.cmd_w)
            # Check if all data is available
            if self.odom_x is not None and self.odom_y is not None  and self.cmd_vx is not None and self.cmd_w is not None and self.imu_yaw is not None:
                print("Record: ", self.odom_x, self.odom_y, self.linear_speed, self.cmd_vx, self.cmd_w, self.imu_yaw)
                # Write data to CSV
                with open(self.path, 'a') as csvfile:  # Use 'a' mode to append
                    csvwriter = csv.writer(csvfile)
                    csvwriter.writerow([self.odom_x, self.odom_y, self.linear_speed, self.cmd_vx, self.cmd_w, self.imu_yaw])
                # Reset data
                self.odom_x = None
                self.odom_y = None
                self.imu_yaw = None

    def cmd_vel_callback(self, msg):
        self.cmd_vx = msg.linear.x
        self.cmd_w = msg.angular.z
    
    def motor_speed_callback(self, msg):
        motor_speed = msg.data[0]
        wheel_speed = motor_speed * self.wheel2motor_ratio
        self.linear_speed = wheel_speed * self.wheel_radius


    def odom_callback(self, msg:Odometry):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y

    def imu_callback(self, msg):
        self.imu_yaw = msg.angular_velocity.z


def main(args=None):
    rclpy.init(args=args)
    node = CSVWritterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
