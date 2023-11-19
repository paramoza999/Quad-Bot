#!/usr/bin/env python3

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
# Define your custom neural networks for the actor, critic, and value functions here
# Implement the networks using PyTorch
class SAC(Node):
    def __init__(self):
        super().__init__("SAC")
      # Create a publisher for combined data
        self.combined_data = []
        self.reward=0
        self.action=[]
        self.state=[]
        self.iteration=1
        self.next_state=[]
        self.agent = Agent() 
        self.load_checkpoint =False
        self.new_session = True 
        self.pole=False

        
 
        self.combined_data_sub = self.create_subscription(Float64MultiArray, '/combined_data',self.x_callback, 10)
        
        self.publisher = self.create_publisher(Float64MultiArray, '/gazebo_joint_controller/commands', 10)

       
        self.combined_data_sub1 = self.create_subscription(Float64MultiArray, '/combined_data1',self.y_callback, 10)
        self.last_print_time = time.time()

        
        

        
        
        
    def x_callback(self, msg):
        # if self.load_checkpoint:
        #       self.agent.load_models()
        # if self.new_session==True:
        #       self.agent.load_models()
        #       self.new_session = False 


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
        kp = 0.0000085  # Adjust the proportional gain as needed
        self.error = T.tensor(command_msg.data, dtype=T.float) - self.current_state
        proportional_output = kp * self.error

        # Derivative Controller
        kd = 0.0000055  #Adjust the derivative gain as needed
        derivative_error = (self.error - self.previous_error)/dt if hasattr(self, 'previous_error') else T.zeros_like(self.error)
        derivative_output = kd * derivative_error

        # Integral Controller
        ki = 0.0000027 # Adjust the integral gain as needed
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

        
           
             

        

        

        



        # if(msg.data[3]>=-0.4 and  msg.data[3]<=0.7 and (msg.data[36]>2.0 or msg.data[37]>2.0)):
        #     self.reward+=5

        # if(msg.data[6]>=-0.4 and  msg.data[0]<=0.7 and (msg.data[36]>2.0 or msg.data[37]>2.0)):
        #     self.reward+=5

        # if(msg.data[9]>=-0.4 and  msg.data[3]<=0.7 and (msg.data[36]>2.0 or msg.data[37]>2.0)):
        #     self.reward+=5
        
        
        # if(msg.data[1]>=0.2 and  msg.data[1]<=2.2 and (msg.data[36]>2.0 or msg.data[37]>2.0)):
        #     self.reward+=5
        
        # if(msg.data[4]>=0.2 and  msg.data[4]<=2.2 and (msg.data[36]>2.0 or msg.data[37]>2.0)):
        #     self.reward+=5

        # if(msg.data[7]>=0.2 and  msg.data[7]<=2.2 and (msg.data[36]>2.0 or msg.data[37]>2.0) ):
        #     self.reward+=5
        
        # if(msg.data[10]>=0.2 and  msg.data[10]<=2.2 and (msg.data[36]>2.0 or msg.data[37]>2.0) ):
        #     self.reward+=5
        
        

        # if(msg.data[2]>=0.5 and  msg.data[2]<=2.0 and (msg.data[36]>2.0 or msg.data[37]>2.0) ):
        #     self.reward+=5
        
        # if(msg.data[5]>=0.5 and  msg.data[5]<=2.0 and (msg.data[36]>2.0 or msg.data[37]>2.0) ):
        #     self.reward+=5
        
        # if(msg.data[8]>=0.5 and  msg.data[8]<=2.0 and (msg.data[36]>2.0 or msg.data[37]>2.0) ):
        #     self.reward+=5

        # if(msg.data[11]>=0.5 and  msg.data[11]<=2.0 and (msg.data[36]>2.0 or msg.data[37]>2.0) ):
        #     self.reward+=5
        
        

            
       
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
         
            




class Agent():
    def __init__(self, alpha=0.00005, beta=0.00007,input_dims=[226],
            env=None, gamma=0.69, n_actions=12, max_size=1000000, tau=0.005,
            layer1_size=256, layer2_size=256, batch_size=512, reward_scale=1.0):
        self.gamma = gamma
        self.tau = tau
        self.memory = ReplayBuffer(max_size, input_dims, n_actions)
        self.batch_size = batch_size
        self.n_actions = n_actions

        self.actor = ActorNetwork(alpha, input_dims, n_actions=n_actions,
                    name='actor')
        self.critic_1 = CriticNetwork(beta, input_dims, n_actions=n_actions,
                    name='critic_1')
        self.critic_2 = CriticNetwork(beta, input_dims, n_actions=n_actions,
                    name='critic_2')
        self.value = ValueNetwork(beta, input_dims, name='value')
        self.target_value = ValueNetwork(beta, input_dims, name='target_value')
   
        self.scale = reward_scale
        self.update_network_parameters(tau=1)
        self.combined_data = []



    def choose_action(self, state):
        state = T.tensor(state, dtype=T.float).to(self.actor.device)

        actions, _ = self.actor.sample_normal(state, reparameterize=False)

        lower_limits = T.tensor([[-1.57, -1.0, 0.45, -1.57, -1.0, 0.45, -1.57, -1.0, 0.45, -1.57, -1.0, 0.45]])
        upper_limits = T.tensor([[1.57, 3.14, 2.35, 1.57, 3.14, 2.35, 1.57, 3.14, 2.35, 1.57, 3.14, 2.35]])

        # Move the limits tensors to the same device as actions
        lower_limits = lower_limits.to(actions.device)
        upper_limits = upper_limits.to(actions.device)

        actions = T.tanh(actions)
        actions = T.clamp(actions, -1.57, 3.14)
        actions = actions * (upper_limits - lower_limits) / 2.0 + (upper_limits + lower_limits) / 2.0
        return actions.cpu().detach().numpy()

    def remember(self, state, action, reward, new_state, done):
        self.memory.store_transition(state, action, reward, new_state, done)

    def update_network_parameters(self, tau=None):
        if tau is None:
            tau = self.tau

        target_value_params = self.target_value.named_parameters()
        value_params = self.value.named_parameters()

        target_value_state_dict = dict(target_value_params)
        value_state_dict = dict(value_params)

        for name in value_state_dict:
            value_state_dict[name] = tau*value_state_dict[name].clone() + \
                    (1-tau)*target_value_state_dict[name].clone()

        self.target_value.load_state_dict(value_state_dict)

    def save_models(self):
        print('.... saving models ....')
        self.actor.save_checkpoint()
        self.value.save_checkpoint()
        self.target_value.save_checkpoint()
        self.critic_1.save_checkpoint()
        self.critic_2.save_checkpoint()

    def load_models(self):
        print('.... loading models ....')
        self.actor.load_checkpoint()
        self.value.load_checkpoint()
        self.target_value.load_checkpoint()
        self.critic_1.load_checkpoint()
        self.critic_2.load_checkpoint()

    def learn(self):
        if self.memory.mem_cntr < self.batch_size:
            return

        state, action, reward, new_state, done = \
                self.memory.sample_buffer(self.batch_size)

        reward = T.tensor(reward, dtype=T.float)
        done = T.tensor(done).to(self.actor.device)
        state_ = T.tensor(new_state, dtype=T.float).to(self.actor.device)
        state = T.tensor(state, dtype=T.float).to(self.actor.device)
        action = T.tensor(action, dtype=T.float).to(self.actor.device)

        value = self.value(state).view(-1)
        value_ = self.target_value(state_).view(-1)
        value_[done] = 0.0

        actions, log_probs = self.actor.sample_normal(state, reparameterize=False)
        log_probs = log_probs.view(-1)
        q1_new_policy = self.critic_1.forward(state, actions)
        q2_new_policy = self.critic_2.forward(state, actions)
        critic_value = T.min(q1_new_policy, q2_new_policy)
        critic_value = critic_value.view(-1)

        self.value.optimizer.zero_grad()
        value_target = critic_value - log_probs
        value_loss = 0.5 * F.mse_loss(value, value_target)
        value_loss.backward(retain_graph=True)
        self.value.optimizer.step()

        actions, log_probs = self.actor.sample_normal(state, reparameterize=True)
        log_probs = log_probs.view(-1)
        q1_new_policy = self.critic_1.forward(state, actions)
        q2_new_policy = self.critic_2.forward(state, actions)
        critic_value = T.min(q1_new_policy, q2_new_policy)
        critic_value = critic_value.view(-1)
        
        actor_loss = log_probs - critic_value
        actor_loss = T.mean(actor_loss)
        self.actor.optimizer.zero_grad()
        actor_loss.backward(retain_graph=True)
        self.actor.optimizer.step()
        reward = reward.to(self.actor.device)
        value_ = value_.to(self.actor.device)

        self.critic_1.optimizer.zero_grad()
        self.critic_2.optimizer.zero_grad()
        q_hat = self.scale*reward + self.gamma*value_
        q1_old_policy = self.critic_1.forward(state, action).view(-1)
        q2_old_policy = self.critic_2.forward(state, action).view(-1)
        critic_1_loss = 0.5 * F.mse_loss(q1_old_policy, q_hat)
        critic_2_loss = 0.5 * F.mse_loss(q2_old_policy, q_hat)

        critic_loss = critic_1_loss + critic_2_loss
        critic_loss.backward()
       
        self.critic_1.optimizer.step()
        self.critic_2.optimizer.step()

        self.update_network_parameters()



    
class ReplayBuffer():
    def __init__(self, max_size, input_shape, n_actions):
        self.mem_size = max_size
        self.mem_cntr = 0
        self.state_memory = np.zeros((self.mem_size, *input_shape))
        self.new_state_memory = np.zeros((self.mem_size, *input_shape))
        self.action_memory = np.zeros((self.mem_size, n_actions))
        self.reward_memory = np.zeros(self.mem_size)
        self.terminal_memory = np.zeros(self.mem_size, dtype=bool)

    def store_transition(self, state, action, reward, state_, done):
        index = self.mem_cntr % self.mem_size

        self.state_memory[index] = state
        self.new_state_memory[index] = state_
        self.action_memory[index] = action
        self.reward_memory[index] = reward
        self.terminal_memory[index] = done

        self.mem_cntr += 1

    def sample_buffer(self, batch_size):
        max_mem = min(self.mem_cntr, self.mem_size)

        batch = np.random.choice(max_mem, batch_size)

        states = self.state_memory[batch]
        states_ = self.new_state_memory[batch]
        actions = self.action_memory[batch]
        rewards = self.reward_memory[batch]
        dones = self.terminal_memory[batch]

        return states, actions, rewards, states_, dones


class CriticNetwork(nn.Module):
    def __init__(self, beta, input_dims, n_actions, fc1_dims=512, fc2_dims=512,
            name='critic', chkpt_dir='/home/param/new_ws/src/p5/src/sac_2layer'):
        super(CriticNetwork, self).__init__()
        self.input_dims = input_dims
        self.fc1_dims = fc1_dims
        self.fc2_dims = fc2_dims
        self.n_actions = n_actions
        self.name = name
        self.checkpoint_dir = chkpt_dir
        self.checkpoint_file = os.path.join(self.checkpoint_dir, name+'_sac')

        self.fc1 = nn.Linear(self.input_dims[0]+n_actions, self.fc1_dims)
        self.fc2 = nn.Linear(self.fc1_dims, self.fc2_dims)
        self.q = nn.Linear(self.fc2_dims, 1)

        self.optimizer = optim.Adam(self.parameters(), lr=beta)
        self.device = T.device('cuda:0' if T.cuda.is_available() else 'cpu')

        self.to(self.device)

    def forward(self, state, action):
        action_value = self.fc1(T.cat([state, action], dim=1))
        action_value = F.relu(action_value)
        action_value = self.fc2(action_value)
        action_value = F.relu(action_value)

        q = self.q(action_value)

        return q

    def save_checkpoint(self):
        T.save(self.state_dict(), self.checkpoint_file)

    def load_checkpoint(self):
        self.load_state_dict(T.load(self.checkpoint_file))

class ValueNetwork(nn.Module):
    def __init__(self, beta, input_dims, fc1_dims=512, fc2_dims=512,
            name='value', chkpt_dir='/home/param/new_ws/src/p5/src/sac_2layer'):
        super(ValueNetwork, self).__init__()
        self.input_dims = input_dims
        self.fc1_dims = fc1_dims
        self.fc2_dims = fc2_dims
        self.name = name
        self.checkpoint_dir = chkpt_dir
        self.checkpoint_file = os.path.join(self.checkpoint_dir, name+'_sac')

        self.fc1 = nn.Linear(*self.input_dims, self.fc1_dims)
        self.fc2 = nn.Linear(self.fc1_dims, fc2_dims)
        self.v = nn.Linear(self.fc2_dims, 1)

        self.optimizer = optim.Adam(self.parameters(), lr=beta)
        self.device = T.device('cuda:0' if T.cuda.is_available() else 'cpu')

        self.to(self.device)

    def forward(self, state):
        state_value = self.fc1(state)
        state_value = F.relu(state_value)
        state_value = self.fc2(state_value)
        state_value = F.relu(state_value)

        v = self.v(state_value)

        return v

    def save_checkpoint(self):
        T.save(self.state_dict(), self.checkpoint_file)

    def load_checkpoint(self):
        self.load_state_dict(T.load(self.checkpoint_file))

class ActorNetwork(nn.Module):
    def __init__(self, alpha, input_dims , fc1_dims=512, 
            fc2_dims=512, n_actions=12, name='actor', 
            chkpt_dir='/home/param/new_ws/src/p5/src/sac_2layer'):
        super(ActorNetwork, self).__init__()
        self.input_dims = input_dims
        self.fc1_dims = fc1_dims
        self.fc2_dims = fc2_dims
        self.n_actions = n_actions
        self.name = name
        self.checkpoint_dir = chkpt_dir
        self.checkpoint_file = os.path.join(self.checkpoint_dir, name+'_sac')
       
        self.reparam_noise = 1e-6

        self.fc1 = nn.Linear(*self.input_dims, self.fc1_dims)
        self.fc2 = nn.Linear(self.fc1_dims, self.fc2_dims)
        self.mu = nn.Linear(self.fc2_dims, self.n_actions)
        self.sigma = nn.Linear(self.fc2_dims, self.n_actions)

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

    def save_checkpoint(self):
        T.save(self.state_dict(), self.checkpoint_file)

    def load_checkpoint(self):
        self.load_state_dict(T.load(self.checkpoint_file))

    





















def main(args=None):
    
    rclpy.init(args=args)
    node = SAC()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

    
    
if __name__ == '__main__':
    main()