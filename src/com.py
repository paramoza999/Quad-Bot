#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
import math
import time 

class JointPositionListener(Node):

    def __init__(self):
        super().__init__('dyn_pos')
        
         # List to accumulate joint data
          # List to accumulate IMU data
        self.laser_data = []  
        self.velocity_x=0
        self.velocity_y=0
        self.velocity_z =0
        self.position_x = 0.0
        self.position_y = 0.0
        self.last_linear_acc_x=0
        self.last_linear_acc_y=0
        self.last_linear_acc_z=0
        
        
        self.last_time=0

        self.combined_data_pub = self.create_publisher(Float64MultiArray, '/combined_data', 10)
        self.timer = self.create_timer(0.045,self.publish_combined_data)

        self.combined_data_pub1 = self.create_publisher(Float64MultiArray, '/combined_data1', 10)
        self.timer1 = self.create_timer(0.06, self.publish_combined_data1)

        
        self.subscription = self.create_subscription(JointState, '/joint_states', self.listener_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu_plugin/out', self.imu_callback, 10)
       
        self.lidar = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
   
    def listener_callback(self, msg):
        # Extend the joint_data list with joint positions, velocities, and efforts
        self.joint_data = [] 
        self.joint_data.extend(msg.position)#0:11
        self.joint_data.extend(msg.velocity)#12:23
        self.joint_data.extend(msg.effort)#24:35

    def imu_callback(self, msg):
        self.imu_data = []  

        
        dt =0.00001

        self.velocity_x += (msg.linear_acceleration.x + self.last_linear_acc_x) * dt / 2.0 
        self.velocity_y += (msg.linear_acceleration.y + self.last_linear_acc_y) * dt / 2.0 
        self.velocity_z += (msg.linear_acceleration.z + self.last_linear_acc_y) * dt / 2.0 
        
        self.position_x += self.velocity_x * dt
        self.position_y += self.velocity_y * dt
        
        # Update the last acceleration values and time for the next iteration
        self.last_linear_acc_x = msg.linear_acceleration.x
        self.last_linear_acc_y = msg.linear_acceleration.y
        self.last_linear_acc_z = msg.linear_acceleration.z


       
        # Extend the imu_data list with linear acceleration, angular velocity, and orientation
        self.imu_data.extend([
            msg.linear_acceleration.x,#36
            msg.linear_acceleration.y,#37
            msg.linear_acceleration.z,#38
            msg.angular_velocity.x,#39
            msg.angular_velocity.y,#40
            msg.angular_velocity.z,#41
            msg.orientation.x,#42
            msg.orientation.y,#43
            msg.orientation.z,#44
            msg.orientation.w,#45
            self.velocity_x,
            self.velocity_y,
            self.velocity_z,
            self.position_x,
            self.position_y,

        ])

    def laser_callback(self, msg):
        self.laser_data = []  
        self.final_laser_data =[value if math.isfinite(value) and value < 12.0 else 100.0 for value in msg.ranges]

        
        

    def publish_combined_data(self):
        self.combined_data=[]
        self.combined_data=Float64MultiArray()

        # Combine joint_data and imu_data to form combined_data
        self.combined_data = self.joint_data + self.imu_data +  self.final_laser_data
        # Create and publish Float64MultiArray message
        self.combined_data_msg = Float64MultiArray()
        self.combined_data_msg.data = self.combined_data
        self.combined_data_pub.publish(self.combined_data_msg)

        # Log the length of combined_data
        x = len(self.combined_data)
        self.get_logger().info('Combined Sensor Data to combined2 (with IMU): %s' % x)


    def publish_combined_data1(self):
        self.combined_data=[]
        self.combined_data=Float64MultiArray()

        # Combine joint_data and imu_data to form combined_data
        self.combined_data = self.joint_data + self.imu_data + self.final_laser_data

        # Create and publish Float64MultiArray messages
        self.combined_data_msg = Float64MultiArray()
        self.combined_data_msg.data = self.combined_data
        self.combined_data_pub1.publish(self.combined_data_msg)

        # Log the length of combined_data
        x = len(self.combined_data)
        self.get_logger().info('Combined Sensor Data to combined1(with IMU): %s' % x)

    

def main(args=None):
    rclpy.init(args=args)
    node = JointPositionListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
