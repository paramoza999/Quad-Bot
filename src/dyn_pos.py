#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image

class JointPositionListener(Node):

    def __init__(self):
        super().__init__('dyn_pos')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10
        )
        self.imu_sub = self.create_subscription(Imu, '/imu_plugin/out', self.imu_callback, 10)
        self.imu_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

        self.publisher = self.create_publisher(Float64MultiArray, '/gazebo_joint_controller/commands', 10)

    def listener_callback(self, msg):
        # Log joint names and positions
        self.get_logger().info('Joint Names: %s' % msg.name)
        self.get_logger().info('Joint Positions: %s' % msg.position)
        self.get_logger().info('Joint Velocities: %s' % msg.velocity)
        self.get_logger().info('Joint efforts: %s' % msg.effort)
        self.get_logger().info('Joint efforts: %s' % msg.effort)


    def imu_callback(self, msg):
        # Log joint names and positions
        self.get_logger().info('Imu: %s' % msg.linear_acceleration)


    def image_callback(self, msg):
        # Log joint names and positions
        self.get_logger().info('Imu: %s' % msg.data)
        
     
        # # Update the joint positions by adding 0.1 to each position
        # updated_positions = [pos for pos in msg.position]
        
        # # Create and publish the command message
        # command_msg = Float64MultiArray()
        # command_msg.data = updated_positions
        # self.publisher.publish(command_msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointPositionListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
