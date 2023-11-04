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
        self.publisher = self.create_publisher(Float64MultiArray, '/gazebo_joint_controller/commands', 10)

        # Initialize an array to hold the combined sensor data
        self.combined_data = []

    def listener_callback(self, msg):
        # Collect joint positions, velocities, and efforts into a single array
        self.combined_data = []
        self.combined_data.extend(msg.position)
        self.combined_data.extend(msg.velocity)
        self.combined_data.extend(msg.effort)

    

    def imu_callback(self, msg):
        # Collect linear acceleration data from the IMU message
        linear_acceleration = [
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ]
        angular_velocity= [
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ]

        orientation= [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z
        ]

        self.combined_data.extend(linear_acceleration)
        self.combined_data.extend(angular_velocity)
        self.combined_data.extend(orientation)

        # Log the updated combined data
        self.get_logger().info('Combined Sensor Data (with IMU): %s' % self.combined_data)

def main(args=None):
    rclpy.init(args=args)
    node = JointPositionListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
