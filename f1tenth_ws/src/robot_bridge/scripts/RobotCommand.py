#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
import numpy as np
# from robot_bridge.mobile_kinematic import Ackerman_Kinematic
from tf_transformations import quaternion_from_euler

class RobotCommandNode(Node):
    def __init__(self):
        super().__init__('robot_command_node')

        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)
        self.create_subscription(Float64MultiArray, "/cmd_steer", self.cmd_steer_callback, 10)
        self.pub_mcu_cmd = self.create_publisher(Float64MultiArray, "/mcu_cmd", 10)
        self.create_timer(0.033, self.timer_callback)

        # self.Kine = Ackerman_Kinematic(r=0.03375, b=0.1625)
        self.qd = np.zeros(2)
        self.pose = np.zeros(3)
        self.twist = np.zeros(2)
        self.quat = quaternion_from_euler(0.0, 0.0, self.pose[2])
        self.lasttimestamp = self.get_clock().now()

        self.wheel_radius = 0.031

        self.wheel2motor_ratio = (27/68) * (15/39)

        self.steer_angle = 0.0
        self.wheel_speed = 0.0
        self.cmd_vel = [0.0, 0.0]

    def timer_callback(self):
        # calculate dt
        currenttimestamp = self.get_clock().now()
        dt = (currenttimestamp - self.lasttimestamp).to_msg().nanosec * 1.0e-9
        self.lasttimestamp = currenttimestamp

        if (self.cmd_vel[0]  >= 3.0):
            self.cmd_vel[0]  = 3.0
        if ( self.cmd_vel[0]  <= -3.0):
            self.cmd_vel[0]  = -3.0



        # update wheel_speed
        self.wheel_speed = self.cmd_vel[0] / self.wheel_radius 
        # update wheel to motor speed
        self.motor_speed = self.wheel2motor(self.wheel_speed, self.wheel2motor_ratio)

        if self.cmd_vel[0] != 0.0:
            self.steer_angle = np.arctan(self.cmd_vel[1]*0.263/self.cmd_vel[0]) * (-1) #0.257 #0.262
        else:
            self.steer_angle = np.arctan(self.cmd_vel[1]*0.263/1.0e-9) * (-1)

        if self.steer_angle >= 0.698:
            self.steer_angle = 0.698
        if self.steer_angle <= -0.698:
            self.steer_angle = -0.698

        # publish cmd_motor
        self.pub_mcu_command(self.steer_angle, self.motor_speed)

    
    def cmd_vel_callback(self, msg):
        self.cmd_vel = [msg.linear.x, msg.angular.z]
    
    def cmd_steer_callback(self, msg):
        self.steer_angle = msg.data[0]


    def wheel2motor(self, qd, ratio):
        return qd/ratio

    def pub_mcu_command(self, steer_angle, motor_speed):

        mcu_cmd = Float64MultiArray()
        mcu_cmd.data = [steer_angle, motor_speed]
        self.pub_mcu_cmd.publish(mcu_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = RobotCommandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
