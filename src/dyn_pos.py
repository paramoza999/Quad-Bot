#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointPositionListener(Node):

    def __init__(self):
        super().__init__('dyn_pos')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('Joint Names: %s' % msg.name)
        self.get_logger().info('Joint Positions: %s' % msg.position)

def main(args=None):
    rclpy.init(args=args)
    node = JointPositionListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
