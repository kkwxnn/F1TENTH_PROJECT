#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
import numpy as np

class ViaPointGenerateNode(Node):
    def __init__(self):
        super().__init__('via_point_generate_node')
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_timer(0.033, self.timer_callback)
        self.lasttimestamp = self.get_clock().now()

        # self.via_point = [[0,-0.157],
        #              [0.2,0.0],
        #              [0,-0.157],
        #              [0.2,0.0],]

        self.via_point = [[0.25,np.pi / 4.0],
                     [0.0,0.0]]
        
        self.time_stamp = [1.0,
                           4.0,]

        # self.via_point = [[0.0,np.pi/2.0],
        #                   [0.0,0.0],
        #                   [0.0,-np.pi/2.0],
        #                   [0.0,0.0],
        #                   [0.0,np.pi/2.0],
        #                   [0.0,0.0],]
        
        # self.time_stamp = [1.0,
        #                    1.0,
        #                    1.0,
        #                    2.0,
        #                    1.0,
        #                    1.0]

        # self.via_point = [[0.1,0.0],
        #                   [0.0,0.0],
        #                   [0.0,np.pi/2.0],
        #                   [0.0,0.0],
        #                   [0.1,0.0],
        #                   [0.0,0.0],
        #                   [0.0,np.pi/2.0],
        #                   [0.0, 0.0],
        #                   [0.1,0.0],
        #                   [0.0,0.0],
        #                   [0.0,np.pi/2.0],
        #                   [0.0,0.0],
        #                   [0.1,0.0],
        #                   [0.0,0.0],
        #                   [0.0,np.pi/2.0],
        #                   [0.0,0.0]]
        
        # self.time_stamp = [1.0,
        #                    2.0,
        #                    1.0,
        #                    1.0,
        #                    1.0,
        #                    10.0,
        #                    1.0,
        #                    1.0,
        #                    1.0,
        #                    2.0,
        #                    1.0,
        #                    1.0,
        #                    1.0,
        #                    10.0,
        #                    1.0,
        #                    1.0,]
        
        self.index = 0
        self.t = 0.0
        self.cmd_vel = [0.0, 0.0]

    def timer_callback(self):
        # calculate dt
        currenttimestamp = self.get_clock().now()
        dt = (currenttimestamp - self.lasttimestamp).to_msg().nanosec * 1.0e-9
        self.lasttimestamp = currenttimestamp
        self.t += dt
        
        if self.index < len(self.time_stamp):
            if self.t >= self.time_stamp[self.index]:
                print(self.via_point)
                self.cmd_vel = self.via_point[self.index]
                self.index += 1
                self.t = 0.0
        else:
            self.cmd_vel = [0.0, 0.0]
        
        self.publish_cmd_vel(self.cmd_vel)


    def publish_cmd_vel(self, vel):
        cmd_vel = Twist()
        cmd_vel.linear.x = vel[0]
        cmd_vel.angular.z = vel[1]
        self.cmd_vel_pub.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    node = ViaPointGenerateNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
