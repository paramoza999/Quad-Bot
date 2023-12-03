#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import torch as T
import torch.nn as nn
import torch.optim as optim


from transforms3d.euler import quat2euler

from sensor_msgs.msg import JointState, Imu, Image
from std_msgs.msg import Float32MultiArray, Int32
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import os
from ament_index_python.packages import get_package_share_directory

import torch.distributions as D
from geometry_msgs.msg import Pose


import os

import torch.nn.functional as F

from torch.distributions.normal import Normal

import time
from gazebo_msgs.srv import DeleteEntity
from gazebo_msgs.srv import SpawnEntity
import xacro
import math

class DQNAgent:
    def __init__(self, input_dims, n_actions, gamma=0.99, epsilon=1.0,
                 epsilon_min=0.01, epsilon_decay=0.995, lr=0.001, batch_size=64,
                 max_mem_size=1000000):
        self.input_dims = input_dims
        self.n_actions = n_actions
        self.gamma = gamma
        self.epsilon = epsilon
        self.epsilon_min = epsilon_min
        self.epsilon_decay = epsilon_decay
        self.lr = lr
        self.batch_size = batch_size
        self.action_space = [i for i in range(n_actions)]
        self.memory = deque(maxlen=max_mem_size)
        self.model = self.build_model()
        self.optimizer = optim.Adam(self.model.parameters(), lr=lr)
        self.loss = nn.MSELoss()
        self.load_checkpoint =False
        self.new_session = True
        self.iteration=1
        self.last_print_time=0

    def build_model(self):
        model = nn.Sequential(
            nn.Linear(*self.input_dims, 128),
            nn.ReLU(),
            nn.Linear(128, 128),
            nn.ReLU(),
            nn.Linear(128, self.n_actions)
        )
        return model

    def choose_action(self, state):
        if np.random.rand() < self.epsilon:
            return random.choice(self.action_space)
        else:
            state = T.tensor(state, dtype=T.float).to(self.model.device)
            actions = self.model(state)
            return T.argmax(actions).item()

    def remember(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))

    def learn(self):
        if len(self.memory) < self.batch_size:
            return

        batch = random.sample(self.memory, self.batch_size)
        states, actions, rewards, next_states, dones = zip(*batch)

        states = T.tensor(states, dtype=T.float).to(self.model.device)
        actions = T.tensor(actions, dtype=T.long).to(self.model.device)
        rewards = T.tensor(rewards, dtype=T.float).to(self.model.device)
        next_states = T.tensor(next_states, dtype=T.float).to(self.model.device)
        dones = T.tensor(dones, dtype=T.float).to(self.model.device)

        self.optimizer.zero_grad()
        current_q = self.model(states).gather(1, actions.view(-1, 1))
        max_next_q = self.model(next_states).max(dim=1)[0].detach()
        target_q = rewards + self.gamma * max_next_q * (1 - dones)
        target_q = target_q.view(-1, 1)

        loss = self.loss(current_q, target_q)
        loss.backward()
        self.optimizer.step()

        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay



class DQNNode(Node):
    def __init__(self):
        super().__init__("DQN")
        self.state = []
        self.action = 0
        self.reward = 0
        self.next_state = []

        self.agent = DQNAgent(input_dims=[230], n_actions=12)

        # ROS subscriptions and publishers
        self.combined_data_sub = self.create_subscription(
            Float64MultiArray, '/combined_data', self.x_callback, 10
        )
        self.publisher = self.create_publisher(
            Float64MultiArray, '/gazebo_joint_controller/commands', 10
        )
        self.combined_data_sub1 = self.create_subscription(
            Float64MultiArray, '/combined_data1', self.y_callback, 10
        )
        # self.lidar_sub = self.create_subscription(
        #     Float64MultiArray, '/lidar_data', self.lidar_callback, 10
        # )

    # def lidar_callback(self, msg):
    #     self.lidar_data = msg.data
    #     self.get_logger().info('Lidar Data: %s' % self.lidar_data)



    def x_callback(self, msg):

        if self.new_session==True:
              self.agent.load_models()
              self.new_session = False


        dt=0.001
        self.state = T.tensor(msg.data, dtype=T.float)

        # Take only the first 12 elements of the current state
        self.current_state = self.state[0:12]

        # Log the current state
        # self.get_logger().info('Combined Sensor Data state data: %s' % self.state)
        # self.get_logger().info('Combined Sensor Data state data: %s' % self.current_state)

        # Choose an action using your agent
        self.action = self.agent.choose_action(T.tensor(self.state, dtype=T.float))

        # Log the chosen action
        # self.get_logger().info('Action values: %s' % self.action)

        # Convert the action to a Float64MultiArray
        command_msg = Float64MultiArray()
        command_msg.data = self.action.flatten().tolist()

        # Proportional Controller
        kp = 0.000008  # Adjust the proportional gain as needed
        self.error = T.tensor(command_msg.data, dtype=T.float) - self.current_state
        proportional_output = kp * self.error

        # Derivative Controller
        kd = 0.0000055  #Adjust the derivative gain as needed
        derivative_error = (self.error - self.previous_error)/dt if hasattr(self, 'previous_error') else T.zeros_like(self.error)
        derivative_output = kd * derivative_error

        # Integral Controller
        ki = 0.000003 # Adjust the integral gain as needed
        if not hasattr(self, 'integral'):
            self.integral = T.zeros_like(self.error)
        self.integral += self.error

        integral_output = ki * self.integral

        # Update the action using proportional, derivative, and integral control
        self.current_state += (proportional_output + derivative_output + integral_output).numpy()
        # Publish the updated action


        updated_command_msg = Float64MultiArray()
        updated_command_msg.data = self.current_state.flatten().tolist()
        self.publisher.publish(updated_command_msg)

        # Save the current error for the next iteration
        self.previous_error = self.error.clone().detach()




    def y_callback(self,msg):

        orientation_data = msg.data[42:46]  # Assuming orientation data is in msg.data[42:46]
        self.iteration+=1
        # Convert orientation data to Euler angles
        euler_angles = quat2euler(orientation_data)
        # self.get_logger().info(f'Euler Angles: {euler_angles}')

        if(self.reward<-14000):
            self.reward=0
            self.agent.save_models()
        #if linear_acc.z<7, reduce rewards
        if(msg.data[38]<8.5):
            self.reward-=10

        if (any(value > 8 for value in msg.data[12:23]) and self.iteration != 100000):
            self.reward -= 10

        else:
            self.reward+=1



        #if orientation rewards for pitch
        if(euler_angles[1]>=0 and euler_angles[1]<=0.6 or (euler_angles[1]<=0 and euler_angles[1]>=-0.4) and ((euler_angles[2]>=-3 and euler_angles[2]<=-2.3) or (euler_angles[2]<=3 and euler_angles[2]>=2.3))):
            self.reward+=1
        else:
            self.reward-=4


        #penalize high angular velocity
        if(msg.data[39]>7 or msg.data[40]>7 or msg.data[41]>7):
            self.reward-=10

        else:
            self.reward+=1


        #if linear_acc.x and linear.acc.y<1, reduce rewards
        if(msg.data[36]<2.0 and msg.data[37]<2.0):
            self.reward-=5



        #if linear_acc.x and linear.acc.y too high, reduce rewards
        if(msg.data[36]>25 or msg.data[37]>25):
            self.reward-=20

        # if sum(value < 12.0 for value in msg.ranges[0:360]) < 17 and sum(value < 12.0 for value in msg.ranges[0:360]) >=1:
        #     self.pole = True

        #  else:
        #     self.pole=False
        #  self.x=sum(value < 12.0 for value in msg.ranges[0:360])
        if (sum(value < 100.0 for value in msg.data[46:227])<=17 and sum(value < 100.0 for value in msg.data[46:227]) >=1):
            self.pole = True
            self.pole_values = [value for value in msg.data[46:227] if value < 12.0]

        else:
            self.pole=False

        if(self.pole==True):
            self.reward+=(3*1/(sum(self.pole_values)+1))
        else:
            self.reward-=1



        if((msg.data[0]>=-0.4 and  msg.data[0]<=0.7)
            and (msg.data[6]>=-0.4 and  msg.data[0]<=0.7)
            and (msg.data[3]>=-0.4 and  msg.data[3]<=0.)
            and (msg.data[9]>=-0.4 and  msg.data[3]<=0.7)
            and (msg.data[1]>=0.2 and  msg.data[1]<=2.2)
            and (msg.data[4]>=0.2 and  msg.data[4]<=2.2)
            and (msg.data[7]>=0.2 and  msg.data[7]<=2.2)
            and (msg.data[10]>=0.2 and msg.data[10]<=2.2)
            and (msg.data[2]>=0.5 and  msg.data[2]<=2.0)
            and (msg.data[5]>=0.5 and  msg.data[5]<=2.0)
            and (msg.data[8]>=0.5 and  msg.data[8]<=2.0)
            and (msg.data[11]>=0.5 and msg.data[11]<=2.0)
            and (msg.data[36]>0.4)):
              self.reward+=1

        if((msg.data[0]>=-0.4 and  msg.data[0]<=0.7)
           and (msg.data[6]>=-0.4 and  msg.data[0]<=0.7)
           and (msg.data[3]>=-0.4 and  msg.data[3]<=0.)
           and (msg.data[9]>=-0.4 and  msg.data[3]<=0.7)
           and (msg.data[1]>=0.2 and  msg.data[1]<=2.2)
           and (msg.data[4]>=0.2 and  msg.data[4]<=2.2)
           and (msg.data[7]>=0.2 and  msg.data[7]<=2.2)
           and (msg.data[10]>=0.2 and msg.data[10]<=2.2)
           and (msg.data[2]>=0.5 and  msg.data[2]<=2.0)
           and (msg.data[5]>=0.5 and  msg.data[5]<=2.0)
           and (msg.data[8]>=0.5 and  msg.data[8]<=2.0)
           and (msg.data[11]>=0.5 and msg.data[11]<=2.0)

           and (msg.data[36]>1.0 )):
             self.reward+=5


        if((msg.data[0]>=-0.4 and  msg.data[0]<=0.7)
           and (msg.data[6]>=-0.4 and  msg.data[0]<=0.7)
           and (msg.data[3]>=-0.4 and  msg.data[3]<=0.)
           and (msg.data[9]>=-0.4 and  msg.data[3]<=0.7)
           and (msg.data[1]>=0.2 and  msg.data[1]<=2.2)
           and (msg.data[4]>=0.2 and  msg.data[4]<=2.2)
           and (msg.data[7]>=0.2 and  msg.data[7]<=2.2)
           and (msg.data[10]>=0.2 and msg.data[10]<=2.2)
           and (msg.data[2]>=0.5 and  msg.data[2]<=2.0)
           and (msg.data[5]>=0.5 and  msg.data[5]<=2.0)
           and (msg.data[8]>=0.5 and  msg.data[8]<=2.0)
           and (msg.data[11]>=0.5 and msg.data[11]<=2.0)

           and (msg.data[36]>3.0 )):
             self.reward+=10


        if((msg.data[0]>=-0.4 and  msg.data[0]<=0.7)
           and (msg.data[6]>=-0.4 and  msg.data[0]<=0.7)
           and (msg.data[3]>=-0.4 and  msg.data[3]<=0.)
           and (msg.data[9]>=-0.4 and  msg.data[3]<=0.7)
           and (msg.data[1]>=0.2 and  msg.data[1]<=2.2)
           and (msg.data[4]>=0.2 and  msg.data[4]<=2.2)
           and (msg.data[7]>=0.2 and  msg.data[7]<=2.2)
           and (msg.data[10]>=0.2 and msg.data[10]<=2.2)
           and (msg.data[2]>=0.5 and  msg.data[2]<=2.0)
           and (msg.data[5]>=0.5 and  msg.data[5]<=2.0)
           and (msg.data[8]>=0.5 and  msg.data[8]<=2.0)
           and (msg.data[11]>=0.5 and msg.data[11]<=2.0)

           and (euler_angles[1]>=0 and euler_angles[1]<=0.6 or
          (euler_angles[1]<=0 and euler_angles[1]>=-0.4) and
          ((euler_angles[2]>=-3 and euler_angles[2]<=-2.3) or
           (euler_angles[2]<=3 and euler_angles[2]>=2.3)))

           and (msg.data[36]>8.0 )):
             self.reward+=300



        if((msg.data[0]>=0.2 and  msg.data[0]<=0.4)
           and (msg.data[6]>=0.2 and  msg.data[0]<=0.4)
           and (msg.data[3]>=0.2 and  msg.data[3]<=0.4)
           and (msg.data[9]>=0.2 and  msg.data[3]<=0.4)
           and (msg.data[1]>=1.0 and  msg.data[1]<=1.4)
           and (msg.data[4]>=1.0  and  msg.data[4]<=1.4)
           and (msg.data[7]>=1.0  and  msg.data[7]<=1.4)
           and (msg.data[10]>=1.0  and msg.data[10]<=1.4)
           and (msg.data[2]>=1.2 and  msg.data[2]<=1.6)
           and (msg.data[5]>=1.2 and  msg.data[5]<=1.6)
           and (msg.data[8]>=1.2 and  msg.data[8]<=1.6)
           and (msg.data[11]>=1.2 and msg.data[11]<=1.6)

           and (euler_angles[1]>=0 and euler_angles[1]<=0.6 or
          (euler_angles[1]<=0 and euler_angles[1]>=-0.4) and
          ((euler_angles[2]>=-3 and euler_angles[2]<=-2.3) or
           (euler_angles[2]<=3 and euler_angles[2]>=2.3)))):
                self.reward+=2

        if((msg.data[0]>=-0.4 and  msg.data[0]<=0.7)
           and (msg.data[6]>=-0.4 and  msg.data[0]<=0.7)
           and (msg.data[3]>=-0.4 and  msg.data[3]<=0.)
           and (msg.data[9]>=-0.4 and  msg.data[3]<=0.7)
           and (msg.data[1]>=0.2 and  msg.data[1]<=2.2)
           and (msg.data[4]>=0.2 and  msg.data[4]<=2.2)
           and (msg.data[7]>=0.2 and  msg.data[7]<=2.2)
           and (msg.data[10]>=0.2 and msg.data[10]<=2.2)
           and (msg.data[2]>=0.5 and  msg.data[2]<=2.0)
           and (msg.data[5]>=0.5 and  msg.data[5]<=2.0)
           and (msg.data[8]>=0.5 and  msg.data[8]<=2.0)
           and (msg.data[11]>=0.5 and msg.data[11]<=2.0)

           and (euler_angles[1]>=0 and euler_angles[1]<=0.6 or
          (euler_angles[1]<=0 and euler_angles[1]>=-0.4) and
          ((euler_angles[2]>=-3 and euler_angles[2]<=-2.3) or
           (euler_angles[2]<=3 and euler_angles[2]>=2.3)))

           and (msg.data[36]>4.0 )):
             self.reward+=100

        if((msg.data[0]>=-0.4 and  msg.data[0]<=0.7)
           and (msg.data[6]>=-0.4 and  msg.data[0]<=0.7)
           and (msg.data[3]>=-0.4 and  msg.data[3]<=0.)
           and (msg.data[9]>=-0.4 and  msg.data[3]<=0.7)
           and (msg.data[1]>=0.2 and  msg.data[1]<=2.2)
           and (msg.data[4]>=0.2 and  msg.data[4]<=2.2)
           and (msg.data[7]>=0.2 and  msg.data[7]<=2.2)
           and (msg.data[10]>=0.2 and msg.data[10]<=2.2)
           and (msg.data[2]>=0.5 and  msg.data[2]<=2.0)
           and (msg.data[5]>=0.5 and  msg.data[5]<=2.0)
           and (msg.data[8]>=0.5 and  msg.data[8]<=2.0)
           and (msg.data[11]>=0.5 and msg.data[11]<=2.0)

           and (euler_angles[1]>=0 and euler_angles[1]<=0.6 or
          (euler_angles[1]<=0 and euler_angles[1]>=-0.4) and
          ((euler_angles[2]>=-3 and euler_angles[2]<=-2.3) or
           (euler_angles[2]<=3 and euler_angles[2]>=2.3)))

           and (msg.data[36]>2 )):
             self.reward+=12

        if((msg.data[0]>=-0.4 and  msg.data[0]<=0.7)
           and (msg.data[6]>=-0.4 and  msg.data[0]<=0.7)
           and (msg.data[3]>=-0.4 and  msg.data[3]<=0.)
           and (msg.data[9]>=-0.4 and  msg.data[3]<=0.7)
           and (msg.data[1]>=0.2 and  msg.data[1]<=2.2)
           and (msg.data[4]>=0.2 and  msg.data[4]<=2.2)
           and (msg.data[7]>=0.2 and  msg.data[7]<=2.2)
           and (msg.data[10]>=0.2 and msg.data[10]<=2.2)
           and (msg.data[2]>=0.5 and  msg.data[2]<=2.0)
           and (msg.data[5]>=0.5 and  msg.data[5]<=2.0)
           and (msg.data[8]>=0.5 and  msg.data[8]<=2.0)
           and (msg.data[11]>=0.5 and msg.data[11]<=2.0)

           and (euler_angles[1]>=0 and euler_angles[1]<=0.6 or
          (euler_angles[1]<=0 and euler_angles[1]>=-0.4) and
          ((euler_angles[2]>=-3 and euler_angles[2]<=-2.3) or
           (euler_angles[2]<=3 and euler_angles[2]>=2.3)))

           and (msg.data[36]>1)):
             self.reward+=3

        if((msg.data[0]>=-0.4 and  msg.data[0]<=0.7)
           and (msg.data[6]>=-0.4 and  msg.data[0]<=0.7)
           and (msg.data[3]>=-0.4 and  msg.data[3]<=0.)
           and (msg.data[9]>=-0.4 and  msg.data[3]<=0.7)
           and (msg.data[1]>=0.2 and  msg.data[1]<=2.2)
           and (msg.data[4]>=0.2 and  msg.data[4]<=2.2)
           and (msg.data[7]>=0.2 and  msg.data[7]<=2.2)
           and (msg.data[10]>=0.2 and msg.data[10]<=2.2)
           and (msg.data[2]>=0.5 and  msg.data[2]<=2.0)
           and (msg.data[5]>=0.5 and  msg.data[5]<=2.0)
           and (msg.data[8]>=0.5 and  msg.data[8]<=2.0)
           and (msg.data[11]>=0.5 and msg.data[11]<=2.0)

           and (euler_angles[1]>=0 and euler_angles[1]<=0.6 or
          (euler_angles[1]<=0 and euler_angles[1]>=-0.4) and
          ((euler_angles[2]>=-3 and euler_angles[2]<=-2.3) or
           (euler_angles[2]<=3 and euler_angles[2]>=2.3)))

           and (msg.data[36]<0.8)):
             self.reward-=5




        self.next_state = msg.data
        current_time = time.time()
        if current_time - self.last_print_time >= 5.0:
            self.get_logger().info('Combined Sensor Data state data: %s' % self.current_state)
            self.get_logger().info('Action values: %s' % self.action)
            self.get_logger().info('Reward values: %s' % self.reward)
            self.get_logger().info('Iteration values: %s' % self.iteration)
            self.get_logger().info('Session_info: %s' % self.new_session)
            self.last_print_time = current_time

        self.next_state = T.tensor(msg.data, dtype=T.float)
        self.agent.remember(self.state, self.action, self.reward, self.next_state, done=False)
        if not self.load_checkpoint:
            self.agent.learn()


        if self.iteration%1000==0:
            self.agent.save_models()


    def save_models(self):
        torch.save(self.model.state_dict(), 'your_model_checkpoint.pth')

    def load_models(self):
        self.model.load_state_dict(torch.load('your_model_checkpoint.pth'))
        self.model.eval()

def main(args=None):
    rclpy.init(args=args)
    node = DQNNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()











# MODEL_NAME='256x2'
# MINIBATCH_SIZE=64
# DISCOUNT=0.99
# MIN_REPLAY_MEMORY_SIZE=1000
# UPDATE_TARGET_EVERY=5
# class DQNAgent:
#     def __init__(self, input_dims, n_actions, gamma=0.99, epsilon=1.0,
#                  epsilon_min=0.01, epsilon_decay=0.995, lr=0.001, batch_size=64,
#                  max_mem_size=1000000):
#         self.model = self.create_model()
#         self.target_model = self.create_model()
#         self.target_model.set_weights(self.model.get_weights())

#         self.replay_memory = deque(maxlen=50000)

#         self.tensorboard = ModifiedTensorBoard(log_dir="logs/{}-{}".format(MODEL_NAME, int(time.time())))


#         self.target_update_counter = 0

#     def build_model(self):
#         model = Sequential()

#         model.add(Conv2D(256, (3, 3), input_shape=env.OBSERVATION_SPACE_VALUES))
#         model.add(Activation('relu'))
#         model.add(MaxPooling2D(pool_size=(2, 2)))
#         model.add(Dropout(0.2))

#         model.add(Conv2D(256, (3, 3)))
#         model.add(Activation('relu'))
#         model.add(MaxPooling2D(pool_size=(2, 2)))
#         model.add(Dropout(0.2))

#         model.add(Flatten())
#         model.add(Dense(64))

#         model.add(Dense(env.ACTION_SPACE_SIZE, activation='linear'))
#         model.compile(loss="mse", optimizer=Adam(lr=0.001), metrics=['accuracy'])
#         return model

#     def update_replay_memory(self, transition):
#         self.replay_memory.append(transition)

#     def get_qs(self, state):
#         return self.model.predict(np.array(state).reshape(-1, *state.shape)/255)[0]