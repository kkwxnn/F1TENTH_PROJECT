#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import (Point, Pose, PoseWithCovariance, Quaternion, Twist, TwistWithCovariance, Vector3, TransformStamped)
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Header, Float32, Float64MultiArray
from tf2_ros import TransformBroadcaster
import tf_transformations
import math
import numpy as np

class YawrateOdom_Ideal(Node):
    def __init__(self):
        super().__init__('YawrateOdom_ideal')
        queue_size = 10
        self.dt_loop = 1/50.0
        # Publisher
        self.publisher_odom_ideal_with_AMCL = self.create_publisher(Odometry, '/odom_ideal_with_AMCL', queue_size)
        self.publisher_odom_ideal = self.create_publisher(Odometry, '/odom_ideal', queue_size)
        self.timer = self.create_timer(self.dt_loop, self.timer_callback)
        self.publisher_imu = self.create_publisher(Float32, '/imu_degree', queue_size)

        # Subscriber
        self.subscription_imu = self.create_subscription(
            Imu,
            '/imu',
            self.feedback_imu,
            queue_size)
        
        self.subscription_motorspeed= self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            queue_size)

        # Initialize the transform broadcaster
        # self.tf_br = TransformBroadcaster(self)
        self.isOdomUpdate = False

        self.pose_cov = np.diag([1.0e-9, 1.0e-9, 1.0e-9, 1.0e-9, 1.0e-9, 1.0e-9])
        self.twist_cov = np.diag([1.0e-9, 1.0e-6, 1.0e-9, 1.0e-9, 1.0e-9, 1.0e-9])

        # Initialize odometry variables
        self.odom_msg = Odometry(
            header=Header(
                stamp=self.get_clock().now().to_msg(),
                frame_id='odom'
            ),
            child_frame_id='base_footprint',
            pose=PoseWithCovariance(
                pose=Pose(
                    position=Point(
                        x=0.0,
                        y=0.0,
                        z=0.0
                    ),
                    orientation=Quaternion(
                        x=0.0,
                        y=0.0,
                        z=0.0,
                        w=1.0
                    )
                ),
                covariance= self.pose_cov.flatten()
            ),
            twist=TwistWithCovariance(
                twist=Twist(
                    linear=Vector3(
                        x=0.0,
                        y=0.0,
                        z=0.0
                    ),
                    angular=Vector3(
                        z=0.0
                    )
                ),
                covariance= self.twist_cov.flatten()
            )
        )

        self.relative_yaw = 0.0
        self.wheelspeed = 0.0
        self.last_callback_time = self.get_clock().now()
        self.initial_orientation = None

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.REAR_STEER_ANGLE = 0.0455

        self.cmd_vel = [0.0, 0.0]

    def cmd_vel_callback(self, msg):
        self.cmd_vel = [msg.linear.x, msg.angular.z]

        
    def feedback_imu(self, msg):
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = tf_transformations.euler_from_quaternion(orientation_list)
        while orientation_list == [0.0, 0.0, 0.0, 0.0] :
            return
        
        if self.initial_orientation is None:
            self.initial_orientation = yaw

        self.relative_yaw = yaw - self.initial_orientation

    def timer_callback(self):
        vx = self.cmd_vel[0] * math.cos(self.relative_yaw) 
        vy = self.cmd_vel[0] * math.sin(self.relative_yaw) 

        self.x += vx * self.dt_loop
        self.y += vy * self.dt_loop

        quaternion = tf_transformations.quaternion_from_euler(0.0, 0.0, self.relative_yaw)

        # Create Odometry message and fill in the data
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()  # Update time stamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        odom_msg.pose.pose = Pose(
            position=Point(x=self.x, y=self.y, z=0.0),
            orientation=Quaternion(
            x=quaternion[0],
            y=quaternion[1],
            z=quaternion[2],
            w=quaternion[3]
        )
        )
        odom_msg.pose.covariance = self.pose_cov.flatten()

        odom_msg.twist.twist.linear = Vector3(x=vx, y=vy, z=0.0)
        odom_msg.twist.twist.angular = Vector3(x=0.0, y=0.0, z=0.0)
        odom_msg.twist.covariance = self.twist_cov.flatten()

        self.publisher_odom_ideal_with_AMCL.publish(odom_msg)

        odom_msg1 = Odometry()
        odom_msg1.header.stamp = self.get_clock().now().to_msg()  # Update time stamp
        odom_msg1.header.frame_id = 'map'
        odom_msg1.child_frame_id = 'base_footprint'
        odom_msg1.pose.pose = Pose(
            position=Point(x=self.x, y=self.y, z=0.0),
            orientation=Quaternion(
            x=quaternion[0],
            y=quaternion[1],
            z=quaternion[2],
            w=quaternion[3]
        )
        )
        odom_msg1.pose.covariance = self.pose_cov.flatten()

        odom_msg1.twist.twist.linear = Vector3(x=vx, y=vy, z=0.0)
        odom_msg1.twist.twist.angular = Vector3(x=0.0, y=0.0, z=0.0)
        odom_msg1.twist.covariance = self.twist_cov.flatten()

        self.publisher_odom.publish(odom_msg1)


        self.publisher_imu.publish(Float32(data=self.relative_yaw*180/math.pi))
        # transform = TransformStamped()
        # transform.header.stamp = odom_msg.header.stamp
        # transform.header.frame_id = 'odom'
        # transform.child_frame_id = 'base_footprint'  # Make sure it matches the child frame ID in odom_output
        # transform.transform.translation.x = odom_msg.pose.pose.position.x
        # transform.transform.translation.y = odom_msg.pose.pose.position.y
        # transform.transform.translation.z = odom_msg.pose.pose.position.z
        # transform.transform.rotation = odom_msg.pose.pose.orientation

        # self.tf_br.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    pub_odom_node = YawrateOdom_Ideal()
    rclpy.spin(pub_odom_node)
    pub_odom_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()