#!/usr/bin/python3


import os
from signal import signal
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from ament_index_python import get_package_share_directory
import sys, yaml
import numpy as np
class CalibrationGen(Node):
    def __init__(self):
        super().__init__('signal_generator')
        
        # establish timer
        self.timer_period = 0.001
        self.sensor_publisher = self.create_publisher(Float64MultiArray,'/sensor_data',10)
        self.timer = self.create_timer(self.timer_period,self.timer_callback)
        
        # load yaml from install folder
        calibration_gen_path = get_package_share_directory('calibration_gen')
        print(calibration_gen_path)
        path = os.path.join(calibration_gen_path, 'config', 'config.yaml')
        with open(path) as f:
            self.properties = yaml.load(f, Loader=yaml.loader.SafeLoader)

    def timer_callback(self):
        msg = Float64MultiArray()
        # generate data and update it to msg
        mean = self.properties['mean']
        cov = self.properties['covariance']
        print(self.properties)
        print(mean)
        print(cov)
        signal = np.random.multivariate_normal(mean, cov, 1)
        msg.data = signal[0].tolist()
        self.sensor_publisher.publish(msg)
    
def main(args=None):
    rclpy.init(args=args)
    node = CalibrationGen()
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
