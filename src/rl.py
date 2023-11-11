import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu, Image
from std_msgs.msg import Float32MultiArray, Int32
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

import torch.distributions as D

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
import random
from collections import deque

# Define your custom neural networks for the actor, critic, and value functions here
# Implement the networks using PyTorch

class SACAgent(Node):
    def __init__(self):
        self.state_dim = 45  
        self.action_dim = 12  
        self.max_action = 1.0 
        self.min_action = -1.0
        self.memory = deque(maxlen=10000)  # Replay buffer
        self.batch_size = 64
        self.combined_data = []


        # ROS2 subscribers for sensor data
        self.joint_states_sub = self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu_plugin/out', self.imu_callback, 10)
        self.camera_sub = self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 10)

        # Initialize your networks and optimizers here
        self.actor = ActorNetwork(self.state_dim, self.action_dim)

    def joint_states_callback(self, msg):
        self.combined_data.extend(msg.position)
        self.combined_data.extend(msg.velocity)
        self.combined_data.extend(msg.effort)

    def imu_callback(self, msg):
        # Process IMU data received from ROS2
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

        self.get_logger().info('Combined Sensor Data (with IMU): %s' % self.combined_data)

        self.actor = ActorNetwork(self.state_dim, self.action_dim)
        self.critic = CriticNetwork(self.state_dim, self.action_dim)








    def select_action(self, state):
        mean, log_std = self.actor(state)
        std = torch.exp(log_std)
        action_distribution = D.Normal(mean, std)
        sampled_action = action_distribution.sample()

        # Clip the sampled action to stay within the joint limits
        sampled_action = torch.clamp(sampled_action, -3.0, 3.0)  # Assuming joint limits are -3 to 3

        return sampled_action





    def train(self):
        if len(self.memory) < self.batch_size:
            return

        # Sample a random minibatch from the replay buffer
        minibatch = random.sample(self.memory, self.batch_size)
        state, action, reward, next_state, done = zip(*minibatch)

        # Implement SAC training steps here





    def update_target_networks(self):
        # Implement soft target network updates here

    def callback_state(self):
        # Combine processed sensor data to create the current state
        state = self.combined_data  # Replace with your actual state processing logic
        action = self.select_action(state)
        new_state=self.combined_data
        reward = self.reward(new_state)
        next_state = np.zeros(self.combined_data)  # Implement next state retrieval
        done = False  # Implement termination condition

        # Store the transition in the replay buffer
        self.memory.append((state, action, reward, next_state, done))

        # Train the agent
        self.train()


class ActorNetwork(nn.Module):
    def __init__(self, state_dim, action_dim):
        super(ActorNetwork, self).__init__()
        self.fc1 = nn.Linear(state_dim, 256)
        self.fc2 = nn.Linear(256, 256)
        self.mean_fc = nn.Linear(256, action_dim)
        self.log_std_fc = nn.Linear(256, action_dim)

    def forward(self, state):
        x = torch.relu(self.fc1(state))
        x = torch.relu(self.fc2(x))
        mean = self.mean_fc(x)
        log_std = self.log_std_fc(x)  # Logarithm of the standard deviation
        return mean, log_std












def main(args=None):
    rclpy.init(args=args)
    node = SACAgent()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
