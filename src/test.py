#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
import time
from transforms3d.euler import quat2euler
import math


class xy(Node):

    def __init__(self):
        super().__init__('xy')
        self.combined_data = []
        self.reward=0

        #self.combined_data_sub = self.create_subscription(LaserScan, '/scan',self.x_callback, 10)
        
        # self.publisher = self.create_publisher(Float64MultiArray, '/gazebo_joint_controller/commands', 10)
        
       
        self.combined_data_sub1 = self.create_subscription(Float64MultiArray, '/combined_data1',self.x_callback, 10)

        self.last_print_time = time.time()
        self.pole=False
        self.x=0


    def x_callback(self,msg):
        
        # self.state=Float64MultiArray()
        # self.state=[]
        # self.state=msg.data
        # # self.get_logger().info('Combined Sensor Data state data: %s' % self.state)
        # self.subset_data = msg.data[0:12]
        # action = [pos + 0.01 for pos in self.subset_data]
        # command_msg = Float64MultiArray()
        # command_msg.data = action
        # self.publisher.publish(command_msg)

        #self.final_data =[value if math.isfinite(value) and value < 12.0 else 50 for value in msg.ranges[0:180]]
        self.get_logger().info('pole: %s' % msg.data[36:51])

            

        # current_time = time.time()
        # if current_time - self.last_print_time >= 5.0:

        #  #number of values less than infinity less than threshold, then its a pole
        #  if (sum(value < 50.0 for value in self.final_data)<=17 and sum(value < 50.0 for value in self.final_data) >=1):
        #     self.pole = True
        #     self.reward+=1
        #  else:
        #     self.pole=False
        #     self.reward-=1
        #  if(self.pole==True):
        #     self.pole_values = [value for value in self.final_data if value < 12.0]
        #  else:
        #     self.pole_values=0
        #  #the values less than infinity
        #  self.x=sum(value < 1000.0 for value in self.final_data)  

        #  self.get_logger().info('pole: %s' % self.pole)
        #  self.get_logger().info('x: %s' % (self.final_data))
        #  self.get_logger().info('x: %s' % (self.pole_values))
        #  self.last_print_time = current_time

         
        #  if self.cube:
        #     dist=
        #     self.reward+=(0.5*(1/))
        #  if hasattr(msg, 'ranges'):
        #     positive_values_and_indices = [(index, value) for index, value in enumerate(msg.ranges) if value >0 and value<12]
        #     self.get_logger().info('Positive values and their indices: %s' % positive_values_and_indices)
        #  else:
        #     self.get_logger().warn('LaserScan message has no attribute "ranges"')
         









         
    # def y_callback(self,msg):
    #     orientation_data = msg.data[42:46]  # Assuming orientation data is in msg.data[42:46]

    #     # Convert orientation data to Euler angles
    #     euler_angles = quat2euler(orientation_data)
    #     self.get_logger().info(f'Euler Angles: {euler_angles}')

    #     self.reward= self.reward+ msg.data[36]+ msg.data[37]
    #     self.next_state = msg.data
    #     # self.get_logger().info('next state: %s' % self.next_state)
    #     # self.get_logger().info('next reward: %s' % self.reward)


     

        

   

def main(args=None):
    rclpy.init(args=args)
    node = xy()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()