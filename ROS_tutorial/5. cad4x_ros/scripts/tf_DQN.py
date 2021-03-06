#!/usr/bin/env python

import rospy

# import Trigger service and custom msgs
from cad4x_ros.srv import image_srv
from cad4x_ros.msg import DQN_to_CNT


# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *

# ros massage libraries
from std_msgs.msg import String, Float32, Bool, Int8, Int64, Char
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

class RGB_sub:
    
    def __init__(self):
        self.bridge = CvBridge()
        self.dst = None
        self.img_sample = None
        self.RGB_sub_00 = rospy.Subscriber("RGB_00", Image, self.callback)
        # self.RGB_sub_01 = rospy.Subscriber("RGB_01", Image, self.callback)
        self.RGB_service = rospy.ServiceProxy('RGB_pending', image_srv)
        
    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)    
        
        rgb_array = np.array(cv_image, dtype=np.float32)
        self.img_sample = cv2.normalize(rgb_array, rgb_array, 0, 1, cv2.NORM_MINMAX)
        self.dst = cv2.resize(self.img_sample, dsize=(100,100), interpolation=cv2.INTER_AREA).astype(np.float32)


class Depth_sub:

    def __init__(self):
        self.bridge = CvBridge()
        self.dst = None
        self.img_sample = None
        self.Depth_sub_00 = rospy.Subscriber("Depth_00", Image, self.callback)
        #self.Depth_sub_00 = rospy.Subscriber("Depth_00", Image, self.callback)
        self.Depth_service = rospy.ServiceProxy('Depth_pending', image_srv)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
        except CvBridgeError as e:
            print(e)    
        
        depth_array = np.array(cv_image, dtype=np.float32)
        self.img_sample = cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)
        self.dst = cv2.resize(self.img_sample, dsize=(100,100), interpolation=cv2.INTER_AREA).astype(np.float32)
        self.dst = self.dst.reshape(100,100,1)

        # print(self.dst)


class data_publisher:
    
    def __init__(self):
        self.n_state = None
        
        # publish the action idx and pending message to action.py
        self.pub_DQN_to_CNT = rospy.Publisher('action_index', DQN_to_CNT, queue_size=10)
        self.pub_reset_00 = rospy.Publisher('Reset_00', Bool, queue_size=1)

class data_subscriber:
    def __init__(self):
        self.DQN_resume = None
        self.reward = None
        self.done = None
        self.reset = None
        
        # Subcribe the resume signal, reward, terimnal from action.py
        self.sub_DQN_resume = rospy.Subscriber('DQN_resume', Int64, self.callback_pending)
        self.sub_reward = rospy.Subscriber('DQN_reward', Float32, self.callback_reward)
        self.sub_terminal = rospy.Subscriber('DQN_terminal', Bool, self.callback_terminal)
        self.sub_reset_00 = rospy.Subscriber('Reset_00', Bool, self.callback_reset)

    def callback_pending(self, data):
        self.DQN_resume = data.data

    def callback_reward(self, data):
        self.reward = data.data
        # print(self.reward)

    def callback_terminal(self, data):
        self.done = data.data
        
    def callback_reset(self, data):
        self.reset = data.data

    
class Model(tf.keras.Model):
    
    def __init__(self, num_actions):
        super(Model, self).__init__()
        self.RGB_conv1 = kl.Conv2D(16, kernel_size=(10,10), strides=2, activation='relu')
        self.RGB_conv2 = kl.Conv2D(32, kernel_size=(5,5), strides=2 , activation='relu')
        self.RGB_conv3 = kl.Conv2D(64, kernel_size=(3,3), strides=2 , activation='relu')
        self.RGB_flat = kl.Flatten()

        self.depth_conv1 = kl.Conv2D(16, kernel_size=(10,10), strides=2, activation='relu')
        self.depth_conv2 = kl.Conv2D(32, kernel_size=(5,5), strides=2, activation='relu')
        self.depth_conv3 = kl.Conv2D(64, kernel_size=(3,3), strides=2, activation='relu')
        self.depth_flat = kl.Flatten()

        self.concatenate = kl.Add()

        self.adv_dense = kl.Dense(512, activation = 'relu')
        self.adv_out = kl.Dense(num_actions)

        self.v_dense = kl.Dense(512, activation = 'relu')
        self.v_out = kl.Dense(1)

        self.lambda_layer = kl.Lambda(lambda x: x - tf.reduce_mean(x))
        self.combine = kl.Add()

        
    def call(self, inputs):
        # x = tf.convert_to_tensor(inputs, dtype=tf.float32)
        x1 = self.RGB_conv1(inputs[0])
        x1 = self.RGB_conv2(x1)
        x1 = self.RGB_conv3(x1)
        x1 = self.RGB_flat(x1)

        x2 = self.depth_conv1(inputs[1])
        x2 = self.depth_conv2(x2)
        x2 = self.depth_conv3(x2)
        x2 = self.depth_flat(x2)

        x = self.concatenate([x1, x2])

        adv = self.adv_dense(x)
        adv = self.adv_out(adv)

        v = self.v_dense(x)
        v = self.v_out(v)

        norm_adv = self.lambda_layer(adv)
        combined = self.combine([v, norm_adv])

        return combined
    
    def action_value(self, obs):
        q_values = self.predict(obs)
        best_action = np.argmax(q_values, axis=-1)
        return best_action[0], q_values[0]
 
class DQNAgent:
    
    def __init__(self, model, target_model, RGB_state, Depth_state, RGB, Depth, pub, sub, num_actions, buffer_size=100, learning_rate=.001, epsilon=.1,
                 epsilon_decay=0.995, min_epsilon=.01, gamma=.9, batch_size=4, target_update_iter=10, train_nums=1000, 
                 start_learning=5):
        
        self.model = model
        self.target_model = target_model
        opt = ko.Adam(learning_rate=learning_rate, clipvalue= 10.0)             # do gredient clip
        self.model.compile(optimizer=opt, loss='mse')
        
        self.RGB_img = RGB
        self.Depth_img = Depth
        self.pub_to_action = pub
        self.sub_from_action = sub
        
        # parameters
        self.state = [RGB_state, Depth_state]           # gazebo environment
        self.lr = learning_rate                         # learning rate
        self.epsilon = epsilon                          # e-greedy when exploring
        self.epsilon_decay = epsilon_decay
        self.min_epsilon = min_epsilon
        self.gamma = gamma                              # discount rate
        self.batch_size = batch_size                    # batch_size
        self.target_update_iter = target_update_iter    # target update period
        self.train_nums = train_nums                    # total training steps
        self.num_in_buffer = 0                          # trasitions num in buffer
        self.buffer_size = buffer_size                  # replay buffer size
        self.start_learning = start_learning            # step to begin learning (save transitions before that step)
        self.num_actions = num_actions                  # number of actions
        self.ep_reward = 0

        # replay buffer
        # buffer_size * W * H * channel (100, 320, 480, 3)
        self.states = [np.empty((self.buffer_size,) + self.state[0].shape), np.empty((self.buffer_size,) + self.state[1].shape)]
        self.actions = np.empty((self.buffer_size), dtype=np.int8)
        self.rewards = np.empty((self.buffer_size), dtype=np.float32)
        self.dones = np.empty((self.buffer_size), dtype=np.bool)
        self.next_states = [np.empty((self.buffer_size,) + self.state[0].shape), np.empty((self.buffer_size,) + self.state[1].shape)]
        self.next_idx = 0
    '''
    def test(self, state):
        obs, done, ep_reward = state, False, 0                                      # initialize the all states
        self.pub.DQN_resume = 0                                                     # reset the penind loop
        while not done:
            obs = state
            # Using [None] to extend its dimension [W, H, C] -> [batch, W, H, C]
            self.pub.act, _ = self.model.action_value(obs[None])
            self.pub.pub_DQN_to_CNT.publish(self.pub.act_msg)
            
            rospy.sleep(0.5)
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
            rospy.sleep(0.5)
            cv2.imshow("Image window", img_sample)
            cv2.waitKey(3)
            

            n_state, reward, done = self.img_send.dst, self.reward, self.done
            done = True
            # ep_reward += reward
            time.sleep(0.05)
        #self.env.close()

    return act
    '''

    def train(self):
        # 1. initialize the initail observation of the agent
        state = [self.RGB_img.dst, self.Depth_img.dst]                                               # bring the ficture from the image_search node
        non_state = [self.RGB_img.dst[None], self.Depth_img.dst[None]] 
        self.target_model.action_value(non_state)
        for t in range(self.train_nums):
        # 2. input the state to the network model
        # Using [None] to extend its dimension [W, H, C] -> [batch, W, H, C]
            best_action, q_values = self.model.action_value(non_state)
        # 3. get the e-greedy action (Max Q or random action)
            action = self.get_action(best_action)
            # print("action", action)     
            
        # 4. reset the pening loop of DQN code 
            self.sub_from_action.DQN_resume = 0 
            rospy.sleep(0.1)
            print("DQN code is Pended")
            
            # pending loop
            while True:             
        # 5. Transfer the resume signal and action index to Action.py when receive the DQN resume signal from action.py
                self.pub_to_action.pub_DQN_to_CNT.publish(action=action, pending = 1)
                # rospy.sleep(0.1)   
                if self.sub_from_action.DQN_resume == 1:
                    break
                
            print("DQN code is resumed")     
           
        # 6. take the action in the env to return s', r, done
            n_state, reward, done = [self.RGB_img.dst, self.Depth_img.dst], self.sub_from_action.reward, self.sub_from_action.done
            
            print(t, reward, done)
            
            # reward += reward            
            #img_sample = self.img_send.img_sample
            #img = cv2.resize(img_sample, dsize=(100,100), interpolation=cv2.INTER_AREA).astype(np.float32)
            cv2.imshow("RGB", self.RGB_img.img_sample)
            cv2.imshow("Depth", self.Depth_img.img_sample)
            cv2.waitKey(1)
            
            # store that transition into replay buffer
            self.store_transition(state, action, reward, n_state, done)
            self.num_in_buffer = min(self.num_in_buffer + 1, self.buffer_size)
            
            # transfer the resume signal of Test.py
            # self.pub.pub_pending_test.publish(self.pub.pending_Test_msg.data)
                   
            if t > self.start_learning:                                 # start learning
                losses = self.train_step()
                if t % 10 == 0:
                    print('losses each 1000 steps: ', losses)
                
            if t % self.target_update_iter == 0:
                self.update_target_model()
                       
            if done == True:
                print("DQN code is pending due to terminal process")
                while (not rospy.is_shutdown()):
                    if self.sub_from_action.reset == False:
                        break
                    
                print("Terminal Process Done")
            else:    
                state = n_state
        
        print("DQN Done")
                
            
    def train_step(self):
        idxes = self.sample(self.batch_size)
        self.s_batch = [self.states[0][idxes], self.states[1][idxes]]
        self.a_batch = self.actions[idxes]
        self.r_batch = self.rewards[idxes]
        self.ns_batch = [self.next_states[0][idxes], self.next_states[1][idxes]]
        self.done_batch = self.dones[idxes]
        
        # Local networks
        target_f = self.model.predict(self.s_batch)
        target_f2 = np.argmax(self.model.predict(self.ns_batch), axis=1)
        # print(target_f2)
        
        #target_q = self.r_batch + self.gamma * \
        #    np.amax(self.get_target_value(self.ns_batch), axis=1) * (1 - self.done_batch)
        
        target_q = self.r_batch + self.gamma * \
            np.array([self.get_target_value(self.ns_batch)[0][target_f2[0]],
            self.get_target_value(self.ns_batch)[1][target_f2[1]],
            self.get_target_value(self.ns_batch)[2][target_f2[2]],
            self.get_target_value(self.ns_batch)[3][target_f2[3]]])

        # print('---')
        # print(np.amax(self.get_target_value(self.ns_batch), axis=1))
        # print('---')

        
        for i, val in enumerate(self.a_batch):
            target_f[i][val] = target_q[i]
            
        losses = self.model.train_on_batch(self.s_batch, target_f)
        
        return losses
    
    '''
    def evaluation(self, state):
        state, done, ep_reward = state, False, 0.0
        # one episode until done
        while not done:
            action, q_values = self.model.action_value(state[None])
            state, reward, done, info = env.step(action)
            ep_reward += reward
            time.sleep(0.05)
        env.close()
        return ep_reward
    '''
    
    # store transitions into replay buffer
    def store_transition(self, obs, action, reward, next_state, done):
        n_idx = self.next_idx % self.buffer_size
        self.states[0][n_idx], self.states[1][n_idx] = obs[0], obs[1]
        self.actions[n_idx] = action
        self.rewards[n_idx] = reward
        self.next_states[0][n_idx], self.next_states[1][n_idx] = next_state[0], next_state[1]
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
            return np.random.randint(0, self.num_actions - 1)
        return best_action
    
    def update_target_model(self):
        print('update_target_model')
        self.target_model.set_weights(self.model.get_weights())
        
    def get_target_value(self, obs):
        return self.target_model.predict(obs)
    
    def e_decay(self):
        self.epsilon *= self.epsilon_decay
        
        
def main():

    # initiate the node
    rospy.init_node('tf_DQN_node', anonymous= True)
        
     # wait for this service to be running
    rospy.wait_for_service('RGB_pending', timeout=None)
    rospy.wait_for_service('Depth_pending', timeout=None)

    # Create the connection to the service.
    RGB_service = rospy.ServiceProxy('RGB_pending', image_srv)
    Depth_service = rospy.ServiceProxy('Depth_pending', image_srv)

    RGB = RGB_sub()
    Depth = Depth_sub()

    pub = data_publisher()
    sub = data_subscriber()
    num_actions = 26                                         # env.action_space.n <= the number of action
    model = Model(num_actions)
    target_model = Model(num_actions)
    
    # load the image
    print("Load the image")
    time.sleep(1.0) 
    
    # initialize the state
    req_RGB = RGB_service('True')
    rqe_Depth = Depth_service('True')
    
    RGB_state = RGB.dst
    Depth_state = Depth.dst
    #print(Depth_state.shape)


    print("initialize the state")
    time.sleep(1.0)
        
    agent = DQNAgent(model, target_model, RGB_state, Depth_state, RGB, Depth, pub, sub, num_actions)
    
    agent.train()
    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

