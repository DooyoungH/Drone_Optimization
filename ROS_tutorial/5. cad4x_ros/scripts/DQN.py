#!/usr/bin/env python

import rospy
import image_search as image

# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *

# talker
from std_msgs.msg import String, Float32, Bool, Int32

import math
import numpy as np

from collections import namedtuple

import torch

import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F


device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
Transition = namedtuple('Transition', ('state','action', 'next_state', 'reward'))

class ReplayMemory(object):
    
    def __init__(self, capacity):
        self.capacity = capacity
        self.memory = []
        self.position = 0
        
    def push(self, *args):
        # save the transition
        if len(self.memory) < self.capacity:
            self.memory.append(None)
        self.memory[self.position] = Transition(*args)
        self.position = (self.position + 1) % self.capacity
        
    def sample(self, batch_size):
        return random.sample(self.memory, batch_size)
    
    def __len__(self):
        return len(self.memory)
    
class DQN(nn.Module):
    
    def __init__(self):
        super(DQN, self).__init__()
        self.conv1 = nn.Conv2d(1, 16, kernel_size = 10, stride = 2) # 480x320 image
        self.conv2 = nn.Conv2d(16, 32, kernel_size = 5, stride = 2)
        self.conv3 = nn.Conv2d(32, 64, kernel_size = 3, stride = 2)
        self.fc1 = nn.Linear(56*36*64, 512)
        self.fc2 = nn.Linear(512, 4)

    def forward(self, x):
        x = F.relu(self.conv1(x))
        x = F.relu(self.conv2(x))
        x = F.relu(self.conv3(x))
        x = F.relu(self.fc1(x.view(x.size(0), -1)))
        out = F.relu(self.fc2(x)) 
        
        return out

class data_subscribe:
    
    def __init__(self):
        self.pending_signal_sub = rospy.Subscriber('pending_signal', Int32, self.callback_pending)
        self.reward_sub = rospy.Subscriber('reward', Float32, self.callback_reward)
        self.count_sub = rospy.Subscriber('count', Int32, self.callback_count)
        self.terminal_sub = rospy.Subscriber('terminal', Bool, self.callback_terminal)
        self.pending = None
        self.reward = None
        self.done = None
        self.count = None
        #self.image_sub = rospy.Subscriber("DQNimage_00", Image, ic.callback)

    def callback_count(self, data):
        self.count = data

    def callback_pending(self, data):
        self.pending = data

    def callback_reward(self, data):
        self.reward = data

    def callback_terminal(self, data):
    #global processing, new_msg, done
    #if not processing:
    #   new_msg = True
        self.done = data


### Hyper-parameters ###

    
BATCH_SIZE = 128
GAMMA = 0.999
EPS_START = 0.9
EPS_END = 0.05
EPS_DECAY = 200
TARGET_UPDATE = 10

policy_net = DQN().to(device)
target_net = DQN().to(device)
target_net.load_state_dict(policy_net.state_dict())
target_net.eval()

optimizer = optim.Adam(policy_net.parameters())
memory = ReplayMemory(10000)

n_actions = len([0, 0, 0, 0])
steps_done = 0
episode_durations = []


### DQN functions ###


def select_action(state):
    global steps_done
    sample = random.random()
    eps_threshold = EPS_END + (EPS_START - EPS_END) * \
        math.exp(-1. * steps_done / EPS_DECAY)
    steps_done += 1
    if sample > eps_threshold:
        with torch.no_grad():
            return policy_net(state).max(1)[1].view(1,1)
    else:
        return torch.tensor([[random.randrange(n_actions)]], device=device, dtype=torch.long)
    
def optimize_model():
    if len(memory) < BATCH_SIZE:
        return
    transitions = memory.sample(BATCH_SIZE)
    batch = Transision(*zip(*transitions))
    
    non_final_mask = torch.tensor(tuple(map(lambda s: s is not None,
                                            batch.next_state)), device=device, dtype=torch.uint8)
    non_final_next_states = torch.cat([s for s in batch.next_state if s is not None])
    
    state_batch = torch.cat(batch.state)
    action_batch = torch.cat(batch.action)
    reward_batch = torch.cat(batch.action)
    
    state_action_values = policy_net(state_batch).gather(1, action_batch)
    
    next_state_values = torch.zeros(BATCH_SIZE, device=device) # initialize
    next_state_values[non_final_mask] = target_net(non_final_next_states).max(1)[0].detach()
    
    # Calculate MAX Q of future
    expected_state_action_values = (next_state_values * GAMMA) + reward_batch
    
    # Calculate Huber loss rate
    loss = F.smooth_l1_loss(state_action_values, expected_state_action_values.unsqeeze(1))

    # optimize the model
    optimizer.zero_grad()
    loss.backward()
    for param in policy_net.parameters():
        param.grad.data.clamp_(-1, 1)
    optimizer.step()


def main():
    
    ic = image.image_converter_00()
    sb = subscribe()
    
    # initiate the node
    rospy.init_node('DQN', anonymous= True)
        
    
    # Publish the action values
    action_pub_00 = rospy.Publisher("Action_00", Int64, queue_size = 1)
    
    # Subscribe the DQN input image
    image_sub_00 = rospy.Subscriber("DQNimage_00", Image, ic.callback)
       
    num_episodes = 1000
    pending = 0
    
    rate = rospy.Rate(20.0)
    
    
    for i_episode in range(num_episodes):
        #env.reset() 
        # # we will use gazebo reset instead of gym.env
        state = rospy.Subscriber("state_img_00", Image, image.image_converter_00.callback).unsqueeze(0)
        # shape : 1 (batch - unsqueeze) x image W x image H x channels
        # action.item() : tensor to value
         
        for t in count():
            # select the action and run
            action = select_action(state)
            action_pub_00.publish(action.item())
            
            # next_state, reward, done, _ = env.step(action.item())
            # pending loop
            while(pending == 0):
                pending = rospy.Subscriber("Pending_signal", Int64, callback_pending)
            
            reward = rospy.Subscriber("reward", Float32, callback_reward) # return the float 32
            done = rospy.Subscriber('done', Bool, callback_done) # return the True and False
            # return the batch size(N) * Image (W,H,C)
            ######  need the arrangement of state #############
            next_state = rospy.Subscriber("state_img_00", Image, image.image_converter_00.callback).unsqueeze(0)
            
            # regenerate the pending loop
            pending = 0
            
            reward = torch.tensor([reward], dtype=torch.float32, device=device)
        
            if not done:
                next_state = torch.tensor(next_state, dtype=torch.float32).unsqueeze(0)
            else:
                next_state = None
            
            memory.push(state, action, next_state, reward)
        
            state = next_state
            optimize_model()
        
            if done:
                episode_durations.append(t + 1)
                break
        
        if i_episode % TARGET_UPDATE == 0:
            target_net.load_state_dict(policy_net.state_dict())
        
        #print(f'Episode: {i_episode}, Episode_durations: {episode_durations[i_episode]}')

    print('Complete')
    #env.close() # change the homepoint command
        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass