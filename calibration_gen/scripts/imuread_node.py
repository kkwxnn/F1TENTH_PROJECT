#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import serial.tools.list_ports
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, TransformStamped
import numpy as np
# from ament_index_python import get_package_share_directory
from tf_transformations import quaternion_from_euler
import os
import sys, yaml
from tf2_ros import TransformBroadcaster
from calibration_gen.euler_angle_estimation import EulerAngle_Estimation
from ament_index_python.packages import get_package_share_directory

class IMUSerialReader(Node):
    def __init__(self):
        super().__init__('imu_serial_reader')
        # self.timer_period = 0.02  # seconds
        # self.timer = self.create_timer(self.timer_period, self.read_and_publish)
        self.create_subscription(Imu, '/imu_raw', self.imu_data_callback, 10)
        self.publisher_imu_cal = self.create_publisher(Imu, 'imu', 10)
        # self.cli = self.create_client(Bool, 'Calibration')
        # while not self.cli.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('service not available, waiting again...') 
        self.pub_tf_br = TransformBroadcaster(self)          
        self.create_timer(0.033, self.timer_callback)                   

        self.isCalibrated = False
        self.path = os.path.join(get_package_share_directory('calibration_gen'), 'config', 'sensor_calibration.yaml')
        with open(self.path, 'r') as file:
            self.value = yaml.safe_load(file)

        self.imu_msg_cal = Imu()
        cov_gyro_np = np.array(self.value['cov gyro'])
        self.imu_msg_cal .angular_velocity_covariance = cov_gyro_np.flatten()
        cov_acc_np = np.array(self.value['cov acc'])
        self.imu_msg_cal .linear_acceleration_covariance = cov_acc_np.flatten()
        self.imu_msg_cal.orientation_covariance = cov_gyro_np.flatten()

        self.quat = quaternion_from_euler(0.0, 0.0, 0.0)
        self.lasttimestamp = self.get_clock().now()

        self.imu_oreintation_x = 0.0
        self.imu_oreintation_y = 0.0
        self.imu_oreintation_z = 0.0
        self.imu_oreintation = EulerAngle_Estimation()

        self.angle = 0.0
    
    def timer_callback(self):
        self.pub_transformation()

    def integrate(self, val, val_integrate, dt):
        val_integrate = val_integrate + val*dt
        return val_integrate
    
    def imu_data_callback(self, msg : Imu):
        
        currenttimestamp = self.get_clock().now()
        dt = (currenttimestamp - self.lasttimestamp).to_msg().nanosec * 1.0e-9
        self.lasttimestamp = currenttimestamp
        # if self.isCalibrated == True:
        self.imu_msg_cal .header.stamp = self.get_clock().now().to_msg()
        self.imu_msg_cal .header.frame_id = "imu"  # Adjust as needed
        # Gyroscope data in rad/s
        gyro_x = np.round(msg.angular_velocity.x - self.value['offset gyro'][0], 1)
        # if np.abs(gyro_x) < 0.01:
        #     gyro_x = 0.0
        gyro_y = np.round(msg.angular_velocity.y - self.value['offset gyro'][1], 1)
        # if np.abs(gyro_y) < 0.01:
        #     gyro_y = 0.0
        gyro_z = np.round(msg.angular_velocity.z - self.value['offset gyro'][2], 1)
        # if np.abs(gyro_z) < 0.01:
        #     gyro_z = 0.0
        self.imu_msg_cal .angular_velocity.x = gyro_x
        self.imu_msg_cal .angular_velocity.y = gyro_y
        self.imu_msg_cal.angular_velocity.z = gyro_z

        oreintation = self.imu_oreintation.compute_angle(gx=gyro_x, gy=gyro_y, gz=gyro_z, dt=dt)
        self.quat = quaternion_from_euler(oreintation[0], oreintation[1], oreintation[2])
        self.imu_msg_cal.orientation = Quaternion(x=self.quat[0], y=self.quat[1], z=self.quat[2], w=self.quat[3])

        # Accelerometer data in m/s^2
        accel_x = msg.linear_acceleration.x - self.value['offset acc'][0]
        accel_y = msg.linear_acceleration.y - self.value['offset acc'][1]
        if np.abs(accel_x) < 1.0:
            accel_x = 0.0
        if np.abs(accel_y) < 1.0:
            accel_y = 0.0
        self.imu_msg_cal .linear_acceleration.x = accel_x
        self.imu_msg_cal .linear_acceleration.y = accel_y
        self.imu_msg_cal .linear_acceleration.z = msg.linear_acceleration.z - self.value['offset acc'][2]

        print(self.imu_msg_cal )
        self.publisher_imu_cal.publish(self.imu_msg_cal )

    def pub_transformation(self):
        tf_stamp = TransformStamped()
        tf_stamp.header.stamp = self.get_clock().now().to_msg()
        tf_stamp.header.frame_id = "odom"
        tf_stamp.child_frame_id = "imu_base_link"
        tf_stamp.transform.rotation = Quaternion(x=self.quat[0], y=self.quat[1], z=self.quat[2], w=self.quat[3])
        self.pub_tf_br.sendTransform(tf_stamp)


def main(args=None):
    rclpy.init(args=args)
    imu_serial_reader = IMUSerialReader()
    rclpy.spin(imu_serial_reader)
    imu_serial_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
