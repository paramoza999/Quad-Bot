#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image
import time

class xy(Node):

    def __init__(self):
        super().__init__('xy')
        self.combined_data = []
        self.reward=0

        self.combined_data_sub = self.create_subscription(Float64MultiArray, '/combined_data',self.x_callback, 10)
        
        self.publisher = self.create_publisher(Float64MultiArray, '/gazebo_joint_controller/commands', 10)
        
       
        self.combined_data_sub1 = self.create_subscription(Float64MultiArray, '/combined_data1',self.y_callback, 10)

        



    def x_callback(self,msg):
        
        self.state=Float64MultiArray()
        self.state=[]
        self.state=msg.data
        self.get_logger().info('Combined Sensor Data state data: %s' % self.state)
        self.subset_data = msg.data[0:12]
        action = [pos + 0.01 for pos in self.subset_data]
        command_msg = Float64MultiArray()
        command_msg.data = action
        self.publisher.publish(command_msg)


    def y_callback(self,msg):
        

        self.reward= self.reward+ msg.data[36]+ msg.data[37]
        self.next_state = msg.data
        self.get_logger().info('next state: %s' % self.next_state)
        self.get_logger().info('next reward: %s' % self.reward)


     

        

   

def main(args=None):
    rclpy.init(args=args)
    node = xy()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()