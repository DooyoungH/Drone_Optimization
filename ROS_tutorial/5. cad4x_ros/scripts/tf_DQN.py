#!/usr/bin/env python

import rospy

# import Trigger service
from cad4x_ros.srv import image_srv

# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *

# ros massage libraries
from std_msgs.msg import String, Float32, Bool, Int64
from sensor_msgs.msg import Image

# python libraries
import math
import time
import numpy as np

# open cv libraries
import cv2
from cv_bridge import CvBridge, CvBridgeError

# tensorflow 2.0
import tensorflow as tf
print(tf.__version__)

import tensorflow.keras.layers as kl
import tensorflow.keras.optimizers as ko

np.random.seed(1)
tf.random.set_seed(1)

    
class Model(tf.keras.Model):
    def __init__(self, num_actions):
        super(Model, self).__init__()
        self.conv1 = kl.Conv2D(16, kernel_size=(10,10), strides= 2, activation='relu')
        self.conv2 = kl.Conv2D(32, kernel_size=(5,5), strides= 2 , activation='relu')
        self.conv3 = kl.Conv2D(64, kernel_size=(3,3), strides= 2 , activation='relu')
        self.flat = kl.Flatten()
        self.fc1 = kl.Dense(512, activation = 'relu')
        self.fc2 = kl.Dense(num_actions)
        
    def call(self, inputs):
        # x = tf.convert_to_tensor(inputs, dtype=tf.float32)
        x = self.conv1(inputs)
        x = self.conv2(x)
        x = self.conv3(x)
        x = self.flat(x)
        x = self.fc1(x)
        x = self.fc2(x)
        return x
    
    def action_value(self, obs):
        q_values = self.predict(obs)
        best_action = np.argmax(q_values, axis=-1)
        return best_action[0], q_values[0]
 
class DQNAgent:
    def __init__(self, model, target_model, state, img_send, pub, buffer_size=4, learning_rate=.001, epsilon=.1,
                 gamma=.9, batch_size=128, target_update_iter=20, train_nums=100, start_learning=10):
        
        self.model = model
        self.target_model = target_model
        opt = ko.Adam(learning_rate=learning_rate, clipvalie= 10.0)             # do gredient clip
        self.model.compile(optimizer=opt, loss='mse')
        
        self.img_send = img_send
        self.pub = pub
        
        # parameters
        self.state = state                              # gazebo environment
        self.lr = learning_rate                         # learning rate
        self.epsilon = epsilon                          # e-greedy when exploring
        self.gamma = gamma                              # discount rate
        self.batch_size = batch_size                    # batch_size
        self.target_update_iter = target_update_iter    # target update period
        self.train_nums = train_nums                    # total training steps
        self.num_in_buffer = 0                          # trasitions num in buffer
        self.buffer_size = buffer_size                  # replay buffer size
        self.start_learning = start_learning            # step to begin learning (save transitions before that step)
        
        # replay buffer
        # buffer_size * W * H * channel (128, 320, 480, 3)
        self.obs = np.empty((self.buffer_size,) + self.state.shape)
        self.actions = np.empty((self.buffer_size), dtype=np.int8)
        self.reward = np.empty((self.buffer_size), dtype=np.float32)
        self.dones = np.empty((self.buffer_size), dtype=np.bool)
        self.next_states = np.empty((self.buffer_size,) + self.state.shape)
        self.next_idx = 0

    def test(self, state):
        obs, done, ep_reward = state, False, 0
        self.pub.DQN_resume = 0
        while not done:
            obs = state
            # Using [None] to extend its dimension [W, H, C] -> [batch, W, H, C]
            self.pub.act, _ = self.model.action_value(obs[None])
            self.pub.pub_action.publish(self.pub.act_msg.data)
            
            rospy.sleep(0.2)
            print(self.pub.DQN_resume)
            print("Pending")
            
            while True:             
                # request the pending msg from the Test.py
                rospy.sleep(0.2)       
                if self.pub.DQN_resume == 1:
                    break
                
            print("resume")
            
            n_state = self.img_send.dst
                        
            img_sample = self.img_send.img_sample
            cv2.imshow("Image window", img_sample)
            cv2.waitKey(3)
            
            rospy.sleep(0.5)
            # transfer the resume signal of Test.py
            self.pub.pub_pending_test.publish(self.pub.pending_Test_msg.data)
            '''
            n_state, reward, done = self.img_send.dst, self.reward, self.done
            done = True
            # ep_reward += reward
            #if render:  # visually
            #    self.env.render()
            time.sleep(0.05)
        #self.env.close()
            '''
        return act
    
    def train(self):
        # initialize the initail observation of the agent
        obs = self.env.reset()
        obs = greyscale(obs)[None]
        
        for t in range(self.train_nums):
            best_action, q_values = self.model.action_value(obs)            # input the state to the network model
            action = self.get_action(best_action)                           # get the e-greedy action (Max Q or random action)
            next_obs, reward, done, info = self.env.step(action)            # take the action in the env to return s', r, done
            next_obs = greyscale(next_obs)[None]                            # store that transition into replay buffer
            self.store_transition(obs, action, reward, next_obs, done)
            self.num_in_buffer += min(self.num_in_buffer + 1, self.buffer_size)
            
            if t < self.start_learning:                                 # start learning
                losses = self.train_step()
                if t % 1000 == 0:
                    print('losses each 1000 steps: ', losses)
                
            if t % self.target_update_iter == 0:
                self.update_target_model()
                
            obs = next_obs
            
    def train_step(self, t):
        idxes = self.sample(self.batch_size)
        self.s_batch = self.obs[idxes]
        self.a_batch = self.actions[idxes]
        self.r_batch = self.rewards[idxes]
        self.ns_batch = self.next_states[idxes]
        self.done_batch = self.dones[idxes]
        
        target_q = self.r_batch + self.gamma * \
            np.amax(self.get_target_value(self.ns_batch), axis=1) * (1 - self.done_batch)
        target_f = self.model.predict(self.s_batch)
        
        for i, val in enumerate(self.a_batch):
            target_f[i][val] = target_q[i]
            
        losses = self.model.train_on_batch(self.s_batch, target_f)
        
        return losses
    
    def store_transition(self, obs, action, reward, next_state, done):
        n_idx = self.next_idx % self.buffer_size
        self.obs[n_idx] = obs
        self.actions[n_idx] = action
        self.rewards[n_idx] = reward
        self.next_states[n_idx] = next_state
        self.dones[n_idx] = done
        self.next_idx = (self.next_idx + 1) % self.buffer_size
        
    def sample(self, n):
        assert n < self.num_in_buffer
        res = []
        while True:
            num = np.random.randint(0, self.num_in_buffer)
            if num not in res:
                res.append(num)
            if len(res) == n:
                break
        return res
    
    # e-greedy
    def get_action(self, best_action):
        if np.random.rand() < self.epsilon:
            return self.env.action_space.sample()
        return best_action
    
    def update_target_model(self):
        print('update_target_model')
        self.target_model.set_weights(self.model.get_weights())
        
    def get_target_value(self, obs):
        return self.target_model.predict(obs)
        
        
class image_send:
    
    def __init__(self):
        self.bridge = CvBridge()
        self.dst = None
        self.img_sample = None
        self.image_sub = image_sub = rospy.Subscriber("DQNimage_00", Image, self.callback)
        self.img_service = rospy.ServiceProxy('image_pending', image_srv)
        
    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            #print(self.cv_image)
        except CvBridgeError as e:
            print(e)    
        
        self.img_sample = cv_image
        self.dst = cv2.resize(cv_image, dsize=(480,320), interpolation=cv2.INTER_AREA).astype(np.float32)
        
class data_publisher:
    
    def __init__(self):
        self.DQN_resume = None
        self.reward = None
        self.done = None
        self.count = None
        self.n_state = None
        
        self.act = 0
        self.act_msg = Int64()
        self.act_msg.data = self.act
        self.pending_Test = 1
        self.pending_Test_msg = Int64()
        self.pending_Test_msg.data = self.pending_Test
      
        # Action publisher
        self.pub_action = rospy.Publisher('action_idx_msg', Int64, queue_size=10)
        self.pub_pending_test = rospy.Publisher('test_resume_msg', Int64, queue_size=10)
        
        # Subcribers
        self.sub_DQN_resume = rospy.Subscriber('DQN_resume', Int64, self.callback_pending)
        self.reward_sub = rospy.Subscriber('reward', Float32, self.callback_reward)
        self.count_sub = rospy.Subscriber('count', Int64, self.callback_count)
        self.terminal_sub = rospy.Subscriber('terminal', Bool, self.callback_terminal)

    def callback_pending(self, data):
        self.DQN_resume = data.data
    
    def callback_count(self, data):
        self.count = data

    def callback_reward(self, data):
        self.reward = data

    def callback_terminal(self, data):
        self.done = data

def main():

    # initiate the node
    rospy.init_node('tf_DQN', anonymous= True)
        
     # wait for this service to be running
    rospy.wait_for_service('image_pending', timeout=None)

    # Create the connection to the service.
    img_service = rospy.ServiceProxy('image_pending', image_srv)

    
    img_send = image_send()
    pub = data_publisher()
    num_actions = 4 #env.action_space.n <= the number of action
    model = Model(num_actions)
    target_model = Model(num_actions)
    
    # load the image
    print("Load the image")
    time.sleep(1.0) 
    
    # initialize the state
    req_img = img_service('True')
    state = img_send.dst

    print("initialize the state")
    time.sleep(1.0)
        
    agent = DQNAgent(model, target_model, state, img_send, pub)
    
    agent.train()
    
    #rate = rospy.Rate(20.0)
    
    '''
    while (not rospy.is_shutdown()):
        state = img_send.dst
        agent.test(state)
    
        rospy.sleep(5.0)

    
    # agent.train()  
    '''

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

