#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Float64MultiArray

class OdometryCalculation(Node):
    def __init__(self):
        super().__init__('odom_node')

        self.create_timer(0.01, self.timer_callback)
        self.create_subscription(Float64MultiArray, "/wheel_vel", self.wheel_vel_callback, 10)

        self.L_REAR_STEER_ANGLE = -0.0455 # rad
        self.R_REAR_STEER_ANGLE = 0.0455 # rad
        self.L_RX = 0.0 # mm
        self.R_RX = 0.0 # mm
        self.L_RY = -0.078 # mm
        self.R_RY = 0.078 # mm
        self.BETA = 0.0 # slip angle
        self.pub_tf_br = TransformBroadcaster(self)
        
        # init parameters
        self.v_rl = 0.0 # rear left linear velocity
        self.v_rr = 0.0 # rear right linear velocity
        self.x_curr = 0.0 # car x position
        self.y_curr = 0.0 # car y position
        self.theta_curr = 0.0 
        self.v_curr = 0.0 # car linear velocity
        self.w_curr = 0.0 # car angular velocity
        self.quat = quaternion_from_euler(0.0, 0.0, self.theta_curr)

        self.x_prev = 0.0
        self.y_prev = 0.0
        self.v_prev = 0.0
        self.w_prev = 0.0
        self.theta_prev = 0.0

        self.prev_time = self.get_clock().now()

    def timer_callback(self):
        # self.Odo2Track()
        self.OdoYawRate()
        self.pub_transformation()

    def wheel_vel_callback(self, msg):
        self.v_rl = msg.data[1]
        self.v_rr = msg.data[0]
        

    def Odo2Track(self):
        dt = (self.get_clock().now() - self.prev_time).to_msg().nanosec * 1.0e-9

        self.x_curr = self.x_prev + (self.v_prev * dt * np.cos(self.BETA + self.theta_prev + (self.w_prev * dt / 2)))
        self.y_curr = self.y_prev + (self.v_prev * dt * np.sin(self.BETA + self.theta_prev + (self.w_prev * dt / 2)))
        self.theta_curr = self.theta_prev + self.w_prev * dt
        self.quat = quaternion_from_euler(0.0, 0.0, self.theta_curr)

        A1 = self.L_RX * self.v_rr * np.sin(self.L_REAR_STEER_ANGLE)
        A2 = self.L_RY * self.v_rr * np.cos(self.L_REAR_STEER_ANGLE)
        A3 = self.R_RX * self.v_rl * np.sin(self.R_REAR_STEER_ANGLE)
        A4 = self.R_RY * self.v_rl * np.cos(self.R_REAR_STEER_ANGLE)

        B1 = self.L_RX * np.sin(self.L_REAR_STEER_ANGLE) * np.cos(self.R_REAR_STEER_ANGLE - self.BETA)
        B2 = self.L_RY * np.cos(self.L_REAR_STEER_ANGLE) * np.cos(self.R_REAR_STEER_ANGLE - self.BETA)
        B3 = self.R_RX * np.sin(self.R_REAR_STEER_ANGLE) * np.cos(self.L_REAR_STEER_ANGLE - self.BETA)
        B4 = self.R_RY * np.cos(self.R_REAR_STEER_ANGLE) * np.cos(self.L_REAR_STEER_ANGLE - self.BETA)

        C1 = self.v_rl * np.cos(self.R_REAR_STEER_ANGLE - self.BETA)
        C2 = self.v_rr * np.cos(self.L_REAR_STEER_ANGLE - self.BETA)

        self.v_curr = (A1 - A2 - A3 + A4) / (B1 - B2 - B3 + B4)
        self.w_curr = (C1 - C2) / (B1 - B2 - B3 + B4)

        self.prev_time = self.get_clock().now()
        self.x_prev = self.x_curr
        self.y_prev = self.y_curr
        self.v_prev = self.v_curr
        self.w_prev = self.w_curr
        self.theta_prev = self.theta_curr
        print('x: ',self.x_curr , 'y: ',self.y_curr)

    def OdoYawRate(self):
        dt = (self.get_clock().now() - self.prev_time).to_msg().nanosec * 1.0e-9

        self.x_curr = self.x_prev + (self.v_prev * dt * np.cos(self.BETA + self.theta_prev + (self.w_prev * dt / 2)))
        self.y_curr = self.y_prev + (self.v_prev * dt * np.sin(self.BETA + self.theta_prev + (self.w_prev * dt / 2)))
        self.theta_curr = self.theta_prev + self.w_prev * dt
        self.quat = quaternion_from_euler(0.0, 0.0, self.theta_curr)

        self.v_curr = (self.v_rl + self.v_rr)/2
        
        self.prev_time = self.get_clock().now()
        self.x_prev = self.x_curr
        self.y_prev = self.y_curr
        self.v_prev = self.v_curr
        self.w_prev = self.w_curr
        self.theta_prev = self.theta_curr
        print('x: ',self.x_curr , 'y: ',self.y_curr)


    def pub_transformation(self):
        tf_stamp = TransformStamped()
        tf_stamp.header.stamp = self.get_clock().now().to_msg()
        tf_stamp.header.frame_id = "odom"
        tf_stamp.child_frame_id = "base_footprint"
        tf_stamp.transform.translation.x = self.x_curr
        tf_stamp.transform.translation.y = self.y_curr
        tf_stamp.transform.rotation = Quaternion(x=self.quat[0], y=self.quat[1], z=self.quat[2], w=self.quat[3])
        self.pub_tf_br.sendTransform(tf_stamp)
    

def main(args=None):
    rclpy.init(args=args)
    node = OdometryCalculation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()