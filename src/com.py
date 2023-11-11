#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu

class JointPositionListener(Node):

    def __init__(self):
        super().__init__('dyn_pos')
        
         # List to accumulate joint data
          # List to accumulate IMU data

        self.combined_data_pub = self.create_publisher(Float64MultiArray, '/combined_data', 10)
        self.timer = self.create_timer(3, self.publish_combined_data)
        self.combined_data_pub1 = self.create_publisher(Float64MultiArray, '/combined_data1', 10)
        self.timer1 = self.create_timer(5, self.publish_combined_data1)
        self.subscription = self.create_subscription(JointState, '/joint_states', self.listener_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu_plugin/out', self.imu_callback, 10)

    def listener_callback(self, msg):
        # Extend the joint_data list with joint positions, velocities, and efforts
        self.joint_data = [] 
        self.joint_data.extend(msg.position)
        self.joint_data.extend(msg.velocity)
        self.joint_data.extend(msg.effort)

    def imu_callback(self, msg):
        self.imu_data = []  
        # Extend the imu_data list with linear acceleration, angular velocity, and orientation
        self.imu_data.extend([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z
        ])

    def publish_combined_data(self):
        self.combined_data=[]

        # Combine joint_data and imu_data to form combined_data
        self.combined_data = self.joint_data + self.imu_data

        # Create and publish Float64MultiArray message
        self.combined_data_msg = Float64MultiArray()
        self.combined_data_msg.data = self.combined_data
        self.combined_data_pub.publish(self.combined_data_msg)

        # Log the length of combined_data
        x = len(self.combined_data)
        self.get_logger().info('Combined Sensor Data (with IMU): %s' % x)


    def publish_combined_data1(self):
        self.combined_data=[]

        # Combine joint_data and imu_data to form combined_data
        self.combined_data = self.joint_data + self.imu_data

        # Create and publish Float64MultiArray messages
        self.combined_data_msg = Float64MultiArray()
        self.combined_data_msg.data = self.combined_data
        self.combined_data_pub1.publish(self.combined_data_msg)

        # Log the length of combined_data
        x = len(self.combined_data)
        self.get_logger().info('Combined Sensor Data (with IMU): %s' % x)

def main(args=None):
    rclpy.init(args=args)
    node = JointPositionListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
