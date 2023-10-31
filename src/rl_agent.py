#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
import random

# Basic DQN Agent (simplified for demonstration)
class DQNAgent:
    def __init__(self, state_size, action_size):
        self.state_size = state_size
        self.action_size = action_size
        self.memory = []
        self.gamma = 0.95    # discount rate
        self.epsilon = 1.0  # exploration rate
        self.epsilon_min = 0.01
        self.epsilon_decay = 0.995
        self.model = self._build_model()

    def _build_model(self):
        # Simple model with one hidden layer
        from keras.models import Sequential
        from keras.layers import Dense
        model = Sequential()
        model.add(Dense(24, input_dim=self.state_size, activation='relu'))
        model.add(Dense(24, activation='relu'))
        model.add(Dense(self.action_size, activation='linear'))
        model.compile(loss='mse', optimizer='adam')
        return model

    def remember(self, state, action, reward, next_state):
        self.memory.append((state, action, reward, next_state))

    def act(self, state):
        if np.random.rand() <= self.epsilon:
            return random.randrange(self.action_size)
        act_values = self.model.predict(state)
        return np.argmax(act_values[0])  # returns action

    def replay(self, batch_size):
        minibatch = random.sample(self.memory, batch_size)
        for state, action, reward, next_state in minibatch:
            target = reward + self.gamma * np.amax(self.model.predict(next_state)[0])
            target_f = self.model.predict(state)
            target_f[0][action] = target
            self.model.train_on_batch(state, target_f)
        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay

    def save(self, name):
        self.model.save_weights(name)

    def load(self, name):
        self.model.load_weights(name)


class QuadrupedRL(Node):
    def __init__(self):
        super().__init__('rl_agent')
        
        self.subscription = self.create_subscription(
            JointState, '/joint_states', self.listener_callback, 10
        )
        self.subscription
        
        self.publisher = self.create_publisher(Float64MultiArray, '/gazebo_joint_controller/commands', 10)
        
        self.agent = DQNAgent(state_size=9, action_size=9)
        
        self.current_state = None
        self.last_state = None
        self.last_action = None

    def listener_callback(self, msg):
        self.current_state = np.array(msg.position)
        
        # This is a placeholder for your reward function
        reward = self.compute_reward(self.current_state)
        
        if self.last_state is not None and self.last_action is not None:
            self.agent.remember(self.last_state, self.last_action, reward, self.current_state)
        
        # Decide on a new action
        action = self.agent.act(self.current_state.reshape(1, -1))
        self.publish_action(action)
        
        # Training the agent
        if len(self.agent.memory) > 32:
            self.agent.replay(32)
        
        self.last_state = self.current_state
        self.last_action = action
        

    def compute_reward(self, state):
        # A very basic reward function that simply rewards forward movement
        # (placeholder, adjust as necessary)
        return state[0]  # Assuming index 0 corresponds to forward movement

    def publish_action(self, action):
        # Convert action to a suitable message and publish
        msg = Float64MultiArray()
        msg.data = [float(action)] * 9
  # Replicating the action value for all joints (adjust as necessary)
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = QuadrupedRL()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
