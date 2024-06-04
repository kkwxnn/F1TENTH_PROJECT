#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
import numpy as np
from tf_transformations import quaternion_from_euler
from sensor_msgs.msg import Imu

class MCUBridgeNode(Node):
    def __init__(self):
        super().__init__('mcu_bridge_node')

        self.create_subscription(Float64MultiArray, "/imu_raw", self.imu_callback, 10)
        self.create_subscription(Float64MultiArray, "/enc_raw", self.enc_callback, 10)

        self.imu_data_publisher = self.create_publisher(Imu, '/imu', 10)
        self.motor_speed_publisher = self.create_publisher(Float64MultiArray, '/motor_speed', 10)
        self.diff_enc = self.create_publisher(Float64MultiArray, '/diff_enc', 10)

        # self.Kine = Ackerman_Kinematic(r=0.03375, b=0.1625)
        self.qd = np.zeros(2)
        self.pose = np.zeros(3)
        self.twist = np.zeros(2)
        self.quat = quaternion_from_euler(0.0, 0.0, self.pose[2])
        self.lasttimestamp = self.get_clock().now()

        self.gyro_cov = np.diag([1.0e-9, 1.0e-9, 1.0e-9])
        self.acc_cov = np.diag([1.0e-9, 1.0e-9, 1.0e-9])
        self.quat_cov = np.diag([1.0e-9, 1.0e-9, 1.0e-9])

        self.prev_motor_position_msg = 0.0

        self.prev_time = self.get_clock().now()

    
    def imu_callback(self, msg):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu"

        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.linear_acceleration.x = msg.data[3] 
        imu_msg.linear_acceleration.y = msg.data[4] 
        imu_msg.linear_acceleration.z = msg.data[5] 
        imu_msg.linear_acceleration_covariance = self.acc_cov.flatten()

        # Gyroscope data in rad/s
        imu_msg.angular_velocity.x = msg.data[0]
        imu_msg.angular_velocity.y = msg.data[1]
        imu_msg.angular_velocity.z = msg.data[2]
        imu_msg.angular_velocity_covariance = self.gyro_cov.flatten()

        imu_msg.orientation.x = msg.data[6]
        imu_msg.orientation.y = msg.data[7]
        imu_msg.orientation.z = msg.data[8]
        imu_msg.orientation.w = msg.data[9]
        imu_msg.orientation_covariance = self.quat_cov.flatten()
        self.imu_data_publisher.publish(imu_msg)     

    def enc_callback(self, msg):
        dt = (self.get_clock().now() - self.prev_time).to_msg().nanosec * 1.0e-9
        motor_speed = (msg.data[0] - self.prev_motor_position_msg)/dt

        diff_enc_msg = Float64MultiArray()
        diff_enc_msg.data = [msg.data[0] - self.prev_motor_position_msg]
        self.diff_enc.publish(diff_enc_msg)

        self.prev_motor_position_msg = msg.data[0]

        motor_speed_msg = Float64MultiArray()
        motor_speed_msg.data = [motor_speed]
        self.motor_speed_publisher.publish(motor_speed_msg)

        self.prev_time = self.get_clock().now()


def main(args=None):
    rclpy.init(args=args)
    node = MCUBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
