#!/usr/bin/python3


import os
from signal import signal
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
# from ament_index_python import get_package_share_directory
from sensor_msgs.msg import Imu
import sys, yaml
import numpy as np
from std_msgs.msg import Bool
from ament_index_python.packages import get_package_share_directory

class CalibrationSensor(Node):
    def __init__(self):
        super().__init__('CalibrationSensor')
        # self.create_subscription(Float64MultiArray, '/sensor_data', self.sensor_data_callback, 10)
        self.create_subscription(Float64MultiArray, '/imu_raw', self.imu_data_callback, 10)
        # self.srv = self.create_service(Bool, 'Calibration', self.status_callback)

        self.collected_data = []

        self.collected_gyro_data = []
        self.collected_acc_data = []
        self.collected_quat_data = []

        self.n = 0
        self.num = 10000

        self.isCalibrated = False

        # calibration_gen_path = get_package_share_directory('calibration_gen')
        # print(calibration_gen_path)
        self.path = os.path.join(get_package_share_directory('calibration_gen'), 'config', 'sensor_calibration.yaml')

    def timer_callback(self):
        pass

    # def status_callback(self, request, response):
    #     if self.isCalibrated == True:
    #         response.result = True
    #     else:
    #         response.result = False
    #     return response

    def save_calibration(self, mean, cov, name):
        
        with open(self.path, 'r') as file:
            value = yaml.safe_load(file)

        mean_list = mean.tolist()
        cov_list = cov.tolist()

        print(mean_list)
        print(cov_list)

        print(value)
        
        value['offset {}'.format(name)] = mean_list
        value['cov {}'.format(name)] = cov_list

        with open(self.path, 'w') as file:
            yaml.dump(value, file)
        
        print("Calibrated", value)

    def imu_data_callback(self, msg):
        if self.n < self.num and self.isCalibrated == False:
            self.n = self.n + 1

            imu_msg = Imu()
            imu_msg.linear_acceleration.x = msg.data[4] # y accel imu
            imu_msg.linear_acceleration.y = (-1) * msg.data[3] # x accel imu
            imu_msg.linear_acceleration.z = msg.data[5] 

            # Gyroscope data in rad/s
            imu_msg.angular_velocity.x = msg.data[0]
            imu_msg.angular_velocity.y = msg.data[1]
            imu_msg.angular_velocity.z = msg.data[2]

            imu_msg.orientation.x = msg.data[6]
            imu_msg.orientation.y = msg.data[7]
            imu_msg.orientation.z = msg.data[8]
            imu_msg.orientation.w = msg.data[9]

            self.collected_acc_data.append([imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z])
            self.collected_gyro_data.append([imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z])
            self.collected_quat_data.append([imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w])
            print("collect data: " + str(self.n)) 
        else:
            if self.isCalibrated == False:
                data_acc_array = np.array(self.collected_acc_data)
                offset_acc = np.mean(data_acc_array, 0)
                cov_acc = np.absolute(np.cov(data_acc_array.T))

                self.save_calibration(offset_acc, cov_acc, 'acc')

                data_gyro_array = np.array(self.collected_gyro_data)
                offset_gyro = np.mean(data_gyro_array, 0)
                cov_gyro = np.absolute(np.cov(data_gyro_array.T))

                self.save_calibration(offset_gyro, cov_gyro, 'gyro')

                data_quat_array = np.array(self.collected_quat_data)
                offset_quat = np.mean(data_quat_array, 0)
                cov_quat = np.absolute(np.cov(data_quat_array.T))

                self.save_calibration(offset_quat, cov_quat, 'quat')
                
                print("=====================")
                print(offset_acc)
                print(cov_acc)
                print("=====================")
                print(offset_gyro)
                print(cov_gyro)
                print("=====================")
                print(offset_quat)
                print(cov_quat)
                self.isCalibrated = True
                exit()
            
    
def main(args=None):
    rclpy.init(args=args)
    print("A")
    node = CalibrationSensor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
