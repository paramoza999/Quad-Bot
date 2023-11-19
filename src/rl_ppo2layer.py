#!/usr/bin/env python3

import os


import numpy as np
import torch as T
import torch.nn as nn
import torch.optim as optim
from torch.distributions.categorical import Categorical


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu, Image
from std_msgs.msg import Float32MultiArray, Int32
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import os
from ament_index_python.packages import get_package_share_directory

import torch.distributions as D
from geometry_msgs.msg import Pose


import numpy as np
import random
from collections import deque
import os
import torch as T
import torch.nn.functional as F
import torch.nn as nn
import torch.optim as optim
from torch.distributions.normal import Normal
import numpy as np
import time
from gazebo_msgs.srv import DeleteEntity
from gazebo_msgs.srv import SpawnEntity
import xacro

from transforms3d.euler import quat2euler
import math


class PPO(Node):
    def __init__(self):
        super().__init__("PPO")
      # Create a publisher for combined data
        self.combined_data = []
        self.reward=0
        self.action=[]
        self.state=[]
        self.iteration=0
        self.next_state=[]
        self.probs=[]
        self.value=[]
        self.agent = Agent() 
        self.load_checkpoint =False

        
 
        self.combined_data_sub = self.create_subscription(Float64MultiArray, '/combined_data',self.x_callback, 10)
        
        self.publisher = self.create_publisher(Float64MultiArray, '/gazebo_joint_controller/commands', 10)

       
        self.combined_data_sub1 = self.create_subscription(Float64MultiArray, '/combined_data1',self.y_callback, 10)
        self.last_print_time = time.time()

        
        

        
        
        
    def x_callback(self, msg):
        if self.load_checkpoint:
            self.agent.load_models()
        dt=0.001
        self.state = T.tensor(msg.data, dtype=T.float)

        # Take only the first 12 elements of the current state
        self.current_state = self.state[0:12]

        # Log the current state
        # self.get_logger().info('Combined Sensor Data state data: %s' % self.state)
        # self.get_logger().info('Combined Sensor Data state data: %s' % self.current_state)

        # Choose an action using your agent
        self.action,self.probs,self.value = self.agent.choose_action(T.tensor(self.state, dtype=T.float))

        # Log the chosen action
        # self.get_logger().info('Action values: %s' % self.action)

        # Convert the action to a Float64MultiArray
        command_msg = Float64MultiArray()
        command_msg.data = self.action.flatten().tolist()

        # Proportional Controller
        kp = 0.00002  # Adjust the proportional gain as needed
        self.error = T.tensor(command_msg.data, dtype=T.float) - self.current_state
        proportional_output = kp * self.error

        # Derivative Controller
        kd = 0.00006# Adjust the derivative gain as needed
        derivative_error = (self.error - self.previous_error)/dt if hasattr(self, 'previous_error') else T.zeros_like(self.error)
        derivative_output = kd * derivative_error

        # Integral Controller
        ki = 0.000008  # Adjust the integral gain as needed
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

        if(self.reward<-50000):
            self.reward=0
            self.agent.save_models()
            # new_pose = Pose()
            # new_pose.orientation.x = 0.001
            # new_pose.orientation.y = 0.001
        
            # new_pose.orientation.w = 0.1 # set the new quaternion orientation
            # request = SetEntityState.Request()
            # request.state.name = 'hyperdog'
            # request.state.pose = new_pose

            # self.set_entity_state_client.call_async(request)
          
       
            # request = DeleteEntity.Request()
            # entity_name="hyperdog"
            # request.name = entity_name
            # self.delete_entity_client.call_async(request)

            # request1 = SpawnEntity.Request()
            
            # request1.name = entity_name
            # pkg_name = 'p5'
            # file_subpath = 'description/hyperdog.urdf.xacro'
            # xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
            # robot_description_raw = xacro.process_file(xacro_file).toxml()
            # request1.xml=robot_description_raw
            # self.spawn_entity_client.call_async(request1)
            # self.agent.load_models()
            # rclpy.spin_until_future_complete(self.node, future)
            # if future.result() is not None:
            #     self.node.get_logger().info(f'Deleted entity: {entity_name}')
            # else:
            #     self.node.get_logger().error(f'Failed to delete entity: {entity_name}')
            

               

        #if linear_acc.z<7, reduce rewards
        if(msg.data[38]<4.5):
            self.reward-=10

        if (any(value > 7 for value in msg.data[12:23]) and self.iteration != 100000):
            self.reward -= 10
        
        else:
            self.reward+=7


        
        #if orientation rewards for pitch
        if(euler_angles[1]>=0 and euler_angles[1]<=0.6 or (euler_angles[1]<=0 and euler_angles[1]>=-0.4) and ((euler_angles[2]>=-3 and euler_angles[2]<=-2.3) or (euler_angles[2]<=3 and euler_angles[2]>=2.3))):
            self.reward+=5
        else:
            self.reward-=5


        #penalize high angular velocity
        if(msg.data[39]>4 or msg.data[40]>4 or msg.data[41]>4):
            self.reward-=10
        
        else:
            self.reward+=5

        
        #if linear_acc.x and linear.acc.y<1, reduce rewards
        if(msg.data[36]<2.0 and msg.data[37]<2.0):
            self.reward-=5
        else:
            self.reward+=5

        #if linear_acc.x and linear.acc.y too high, reduce rewards
        if(msg.data[36]>25 or msg.data[37]>25):
            self.reward-=7
        
     

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

           and (msg.data[36]>2.5 or msg.data[37]>2.5)):
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

           and (msg.data[36]>3.5 or msg.data[37]>3.5)):
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

           and (msg.data[36]>5.0 or msg.data[37]>5.0)):
             self.reward+=10

        
        

            
       
        self.next_state = msg.data
        current_time = time.time()
        if current_time - self.last_print_time >= 5.0:
            self.get_logger().info('Combined Sensor Data state data: %s' % self.current_state)
            self.get_logger().info('Action values: %s' % self.action)
            self.get_logger().info('Reward values: %s' % self.reward)
            self.get_logger().info('Iteration values: %s' % self.iteration)
            self.last_print_time = current_time

        self.next_state = T.tensor(msg.data, dtype=T.float)

        self.agent.remember(self.state, self.action, self.probs,self.value,self.reward, done=False)
        if not self.load_checkpoint:
            self.agent.learn()
         
            self.agent.save_models()







class PPOMemory:
    def __init__(self, batch_size):
        
        self.probs = []
        self.vals = []
        self.actions = []
        self.rewards = []
        self.dones = []
        self.states=[]

        self.batch_size = batch_size

    def generate_batches(self):
        n_states = len(self.states)
        batch_start = np.arange(0, n_states, self.batch_size)
        indices = np.arange(n_states, dtype=np.int64)
        np.random.shuffle(indices)
        batches = [indices[i:i+self.batch_size] for i in batch_start]

        return np.array(self.states),\
                np.array(self.actions),\
                np.array(self.probs),\
                np.array(self.vals),\
                np.array(self.rewards),\
                np.array(self.dones),\
                batches

    def store_memory(self, state, action, probs, vals, reward, done):
        self.states.append(state)
        self.actions.append(action)
        self.probs.append(probs)
        self.vals.append(vals)
        self.rewards.append(reward)
        self.dones.append(done)

    def clear_memory(self):
        self.states = []
        self.probs = []
        self.actions = []
        self.rewards = []
        self.dones = []
        self.vals = []

class ActorNetwork(nn.Module):
    def __init__(self, n_actions, input_dims, alpha,
            fc1_dims=256, fc2_dims=256, chkpt_dir='/home/param/new_ws/src/p5/src/ppo'):
        super(ActorNetwork, self).__init__()

        self.checkpoint_file = os.path.join(chkpt_dir, 'actor_torch_ppo')
        self.actor = nn.Sequential(
                nn.Linear(*input_dims, fc1_dims),
                nn.ReLU(),
                nn.Linear(fc1_dims, fc2_dims),
                nn.ReLU(),
                nn.Linear(fc2_dims, n_actions),
                nn.Softmax(dim=-1)
        )

        self.optimizer = optim.Adam(self.parameters(), lr=alpha)
        self.device = T.device('cuda:0' if T.cuda.is_available() else 'cpu')
        self.to(self.device)

    def forward(self, state):
        prob = self.fc1(state)
        prob = F.relu(prob)
        prob = self.fc2(prob)
        prob = F.relu(prob)

        mu = self.mu(prob)
        sigma = self.sigma(prob)

        # sigma = T.exp(sigma) + 1e-5  # Add a small positive value to avoid values close to zero
        # sigma = T.clamp(sigma, min=-1, max=2)
        sigma = T.clamp(sigma, min=self.reparam_noise, max=1)

     
        return mu, sigma

    def save_checkpoint(self):
        T.save(self.state_dict(), self.checkpoint_file)

    def load_checkpoint(self):
        self.load_state_dict(T.load(self.checkpoint_file))

class CriticNetwork(nn.Module):
    def __init__(self, input_dims, alpha, fc1_dims=256, fc2_dims=256,
            chkpt_dir='/home/param/new_ws/src/p5/src/sac'):
        super(CriticNetwork, self).__init__()

        self.checkpoint_file = os.path.join(chkpt_dir, 'critic_torch_ppo')
        self.critic = nn.Sequential(
                nn.Linear(*input_dims, fc1_dims),
                nn.ReLU(),
                nn.Linear(fc1_dims, fc2_dims),
                nn.ReLU(),
                nn.Linear(fc2_dims, 1)
        )

        self.optimizer = optim.Adam(self.parameters(), lr=alpha)
        self.device = T.device('cuda:0' if T.cuda.is_available() else 'cpu')
        self.to(self.device)

    def forward(self, state):
        value = self.critic(state)

        return value

    def save_checkpoint(self):
        T.save(self.state_dict(), self.checkpoint_file)

    def load_checkpoint(self):
        self.load_state_dict(T.load(self.checkpoint_file))

class Agent:
    def __init__(self, n_actions=12, input_dims=[46], gamma=0.99, alpha=0.0003, gae_lambda=0.95,
            policy_clip=0.2, batch_size=256, n_epochs=10):
        self.gamma = gamma
        self.policy_clip = policy_clip
        self.n_epochs = n_epochs
        self.gae_lambda = gae_lambda

        self.actor = ActorNetwork(n_actions, input_dims, alpha)
        self.critic = CriticNetwork(input_dims, alpha)
        self.memory = PPOMemory(batch_size)
       
    def remember(self, state, action, probs, vals, reward, done):
        self.memory.store_memory(state, action, probs, vals, reward, done)

    def save_models(self):
        print('... saving models ...')
        self.actor.save_checkpoint()
        self.critic.save_checkpoint()

    def load_models(self):
        print('... loading models ...')
        self.actor.load_checkpoint()
        self.critic.load_checkpoint()

    def choose_action(self, state):
        state = T.tensor(state, dtype=T.float).to(self.actor.device)

        actions, log_probs = self.actor.sample_normal(state, reparameterize=False)

        lower_limits = T.tensor([[-1.57, -1.0, 0.45, -1.57, -1.0, 0.45, -1.57, -1.0, 0.45, -1.57, -1.0, 0.45]])
        upper_limits = T.tensor([[1.57, 3.14, 2.35, 1.57, 3.14, 2.35, 1.57, 3.14, 2.35, 1.57, 3.14, 2.35]])

        # Move the limits tensors to the same device as actions
        lower_limits = lower_limits.to(actions.device)
        upper_limits = upper_limits.to(actions.device)

        actions = actions * (upper_limits - lower_limits) / 2.0 + (upper_limits + lower_limits) / 2.0
        

        
        value = self.critic(state)
        # Move tensors to CPU before converting to NumPy
      
        self.probs = log_probs.cpu().detach().numpy()
        value = value.cpu().detach().numpy()

        return self.probs, value,actions.cpu().detach().numpy()



    def learn(self):
        for _ in range(self.n_epochs):
            state_arr, action_arr, old_prob_arr, vals_arr,\
            reward_arr, dones_arr, batches = \
                    self.memory.generate_batches()

            values = vals_arr
            advantage = np.zeros(len(reward_arr), dtype=np.float32)

            for t in range(len(reward_arr)-1):
                discount = 1
                a_t = 0
                for k in range(t, len(reward_arr)-1):
                    a_t += discount*(reward_arr[k] + self.gamma*values[k+1]*\
                            (1-int(dones_arr[k])) - values[k])
                    discount *= self.gamma*self.gae_lambda
                advantage[t] = a_t
            advantage = T.tensor(advantage).to(self.actor.device)

            values = T.tensor(values).to(self.actor.device)
            for batch in batches:
                states = T.tensor(state_arr[batch], dtype=T.float).to(self.actor.device)
                old_probs = T.tensor(old_prob_arr[batch]).to(self.actor.device)
                actions = T.tensor(action_arr[batch]).to(self.actor.device)

                dist = self.actor(states)
                critic_value = self.critic(states)

                critic_value = T.squeeze(critic_value)

                new_probs = self.probs
                prob_ratio = new_probs.exp() / old_probs.exp()
                #prob_ratio = (new_probs - old_probs).exp()
                weighted_probs = advantage[batch] * prob_ratio
                weighted_clipped_probs = T.clamp(prob_ratio, 1-self.policy_clip,
                        1+self.policy_clip)*advantage[batch]
                actor_loss = -T.min(weighted_probs, weighted_clipped_probs).mean()

                returns = advantage[batch] + values[batch]
                critic_loss = (returns-critic_value)**2
                critic_loss = critic_loss.mean()

                total_loss = actor_loss + 0.5*critic_loss
                self.actor.optimizer.zero_grad()
                self.critic.optimizer.zero_grad()
                total_loss.backward()
                self.actor.optimizer.step()
                self.critic.optimizer.step()

        self.memory.clear_memory()         

    def sample_normal(self, state, reparameterize=True):
            mu, sigma = self.forward(state)
            probabilities = Normal(mu, sigma)

            if reparameterize:
                actions = probabilities.rsample()
            else:
                actions = probabilities.sample()

            action = T.tanh(actions).to(self.device)
            log_probs = probabilities.log_prob(actions)
            log_probs -= T.log(1-action.pow(2)+self.reparam_noise)
            log_probs = log_probs.sum(-1, keepdim=True)

            return action, log_probs      





def main(args=None):
    
    rclpy.init(args=args)
    node = PPO()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

    
    
if __name__ == '__main__':
    main()