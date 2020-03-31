#!/usr/bin/env python

import rospy
import multi_off_board as mob
import numpy as np
# import image_search as image

# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *

# messages and services
from std_msgs.msg import String, Bool, Float32, Int64, Int8
from sensor_msgs.msg  import Image, LaserScan
from cad4x_ros.msg import DQN_to_CNT

from std_srvs.srv import Empty

class data_publisher:
    
    def __init__(self):
        # Initialize the publisher message
        self.pend = 1
        self.pending_msg = Int64()
        self.pending_msg.data = self.pend
        self.n_state = None
        self.n_state_msg = Image()
        self.n_state_msg.data = self.n_state  
        self.reward = None
        self.reward_msg = Float32()
        self.reward_msg.data = self.reward
        # self.done = False
        # self.done_msg = Bool()
        # self.done_msg.data = self.done

        # set up the publisher
        self.DQN_resume = rospy.Publisher('DQN_resume', Int64, queue_size=5)
        self.DQN_reward = rospy.Publisher('reward', Float32, queue_size=5)
        self.done = rospy.Publisher('terminal', Bool, queue_size=5)
        
        self.pub_reset_00 = rospy.Publisher('Reset_00', Bool, queue_size=1)

        # Setpoint publisher    
        self.target_00 = rospy.Publisher('Target_00', PositionTarget, queue_size=1)
        self.target_01 = rospy.Publisher('Target_01', PositionTarget, queue_size=1)
        self.target_02 = rospy.Publisher('Target_02', PositionTarget, queue_size=1)
        
        
class data_subcriber:
    
    def __init__(self):
        # Initialize the subscriber messages
        self.Test_resume = None
        self.action_index = None
        self.laser_array = None
        self.reset = False
        
        # set up the subscriber
        self.dqn_action_idx = rospy.Subscriber('action_idx_msg', DQN_to_CNT, self.action_callback)
        self.sub_laser_00 = rospy.Subscriber('/uav0/laser/scan', LaserScan, self.callback_laser)
        
        self.sub_reset_00 = rospy.Subscriber('Reset_00', Bool, self.callback_reset)
        
    # Callback functions
    def action_callback(self, data):
        self.action_index = data.action
        self.Test_resume = data.pending
        
    def callback_reset(self, data):
        self.reset = data.data
    
    def callback_laser(self, data):
        self.laser_array = [
            min(min(data.ranges[0:19]), 3),
            min(min(data.ranges[20:39]), 3),
            min(min(data.ranges[40:59]), 3),
            min(min(data.ranges[60:79]), 3),
            min(min(data.ranges[80:99]), 3),
            min(min(data.ranges[100:119]), 3),
        ]


# Main function
def main():

    # initiate node
    rospy.init_node('action_node', anonymous=True)
    
    # flight mode object
    # mob = multi_off_board
    # importing the FCU controller
    modes_00 = mob.fcuModes_00()
    modes_01 = mob.fcuModes_01()
    modes_02 = mob.fcuModes_02()
    
    # importing the "Main" controller object
    cnt_00 = mob.Controller()
    cnt_01 = mob.Controller()
    cnt_02 = mob.Controller()
    
    pub_to_dqn = data_publisher()
    sub_from_dqn = data_subcriber()
    
    # importing the off_board object 
    board = mob.off_board(modes_00, modes_01, modes_02, cnt_00, cnt_01, cnt_02)

    print("moving the drone using DQN")    
    
    while (not rospy.is_shutdown()):

        # 1. initialize the pending loop of controller.py
        sub_from_dqn.Test_resume = 0
        rospy.sleep(0.1)
        # 2. pending the action sequence
        print("controller node is pended")
        while True:             
        # 3. request the pending msg from the tf_DQN.py
            rospy.sleep(0.1)
            # this while loop is broken, when reciving the pending message from DQN node       
            if sub_from_dqn.Test_resume == 1:
                print("controller node is resumed")
                break
    
        while (not rospy.is_shutdown()):    
            # positive x - Check OK
            if sub_from_dqn.action_index == 0:
                print("select the x-axis")
                board.controller_00.x_dir()
                pub_to_dqn.target_00.publish(board.controller_00.sp)
                print(sub_from_dqn.laser_array)
                while (not rospy.is_shutdown()): 
                    
                    if (abs(board.controller_00.sp.position.x - board.controller_00.local_pos.x) <= 0.2 and \
                        sub_from_dqn.reset == False):
                        break
                    if np.amin(sub_from_dqn.laser_array) < 3:
                        board.controller_00.stop()
                        pub_to_dqn.target_00.publish(board.controller_00.sp)
                        pub_to_dqn.pub_reset_00.publish(data=True)
                        sub_from_dqn.reset = True
                        break
                    if -1.0 >= board.controller_00.sp.position.z or board.controller_00.sp.position.z >= 80.0:
                        board.controller_00.stop()
                        pub_to_dqn.target_00.publish(board.controller_00.sp)
                        pub_to_dqn.pub_reset_00.publish(data=True)
                        sub_from_dqn.reset = True
                        break

                    continue
                if sub_from_dqn.reset == False:
                
                # 4. one time, publish the reward to DQN code
                    pub_to_dqn.reward_msg.data = 0.0
                    pub_to_dqn.DQN_reward.publish(pub_to_dqn.reward_msg)
                    rospy.sleep(0.1)
                # 5. one time, publish the done message to DQN code
                    pub_to_dqn.done.publish(data=False)
                    #if current position is terminal, action node is publishing the true signal
                
                    pub_to_dqn.pending_msg = 1
                # 6. publish the pending signal message to DQN code
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)
                        
                    rospy.sleep(0.1)
                    pub_to_dqn.pending_msg = 0
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)                                                                        
                    break
                
                else:
                    # When simulater is restarted because of the obstacles, the agents gain negative rewards 
                    pub_to_dqn.reward_msg.data = -100.0
                    pub_to_dqn.DQN_reward.publish(pub_to_dqn.reward_msg)
                    rospy.sleep(0.1)
                    
                    # Then action py publish the done message to DQN code
                    pub_to_dqn.done.publish(data=True)
                    
                    pub_to_dqn.pending_msg = 1
                    # publish the pending signal message to DQN code
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)
                        
                    rospy.sleep(0.1)
                    pub_to_dqn.pending_msg = 0
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)

                    board.controller_00.origin()
                    pub_to_dqn.target_00.publish(board.controller_00.sp)  
                    
                    sub_from_dqn.reset = False
                    # finally pending the action py code until initial states of drones
                    while (not rospy.is_shutdown()):
                        if (board.controller_00.local_pos.x <= 0.1 and board.controller_00.local_pos.x >= -0.1) and \
                            (board.controller_00.local_pos.y <= 0.1 and board.controller_00.local_pos.y >= -0.1) and \
                            (board.controller_00.local_pos.z <= 3.1 and board.controller_00.local_pos.z >= 2.9):
                            pub_to_dqn.pub_reset_00.publish(data=False)
                            break
                    break
                    
            # positive y - Check OK
            if sub_from_dqn.action_index == 1:
                print("select the y-axis")
                board.controller_00.y_dir()
                pub_to_dqn.target_00.publish(board.controller_00.sp)
                        
                while (not rospy.is_shutdown()):
                    
                    if (abs(board.controller_00.sp.position.y - board.controller_00.local_pos.y) <= 0.2 and \
                        sub_from_dqn.reset == False):
                        break
                    if np.amin(sub_from_dqn.laser_array) < 3:
                        board.controller_00.stop()
                        pub_to_dqn.target_00.publish(board.controller_00.sp)
                        pub_to_dqn.pub_reset_00.publish(data=True)
                        sub_from_dqn.reset = True               
                        break
                    if -1.0 >= board.controller_00.sp.position.z or board.controller_00.sp.position.z >= 80.0:
                        board.controller_00.stop()
                        pub_to_dqn.target_00.publish(board.controller_00.sp)
                        pub_to_dqn.pub_reset_00.publish(data=True)
                        sub_from_dqn.reset = True
                        break
                    
                    continue
                
                if sub_from_dqn.reset == False:
                
                # 4. one time, publish the reward to DQN code
                # pub_to_dqn.pub_reward.publish(pub_to_dqn.reward_msg)
                    pub_to_dqn.reward_msg.data = 1.0
                    pub_to_dqn.DQN_reward.publish(pub_to_dqn.reward_msg)
                    #pub_to_dqn.reward.publish(data=1.0)
                # 5. one time, publish the done message to DQN code
                    pub_to_dqn.done.publish(data=False)
                
                    pub_to_dqn.pending_msg = 1
                # 6. publish the pending signal message to DQN code
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)
                    #if current position is terminal, action node is publishing the true signal
                        
                    rospy.sleep(0.1)
                    pub_to_dqn.pending_msg = 0
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)                                                                   
                    break
                
                else:
                    # When simulater is restarted because of the obstacles, the agents gain negative rewards 
                    pub_to_dqn.reward_msg.data = -100.0
                    pub_to_dqn.DQN_reward.publish(pub_to_dqn.reward_msg)
                    rospy.sleep(0.1)
                    
                    # Then action py publish the done message to DQN code
                    pub_to_dqn.done.publish(data=True)
                    
                    pub_to_dqn.pending_msg = 1
                    # publish the pending signal message to DQN code
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)
                        
                    rospy.sleep(0.1)
                    pub_to_dqn.pending_msg = 0
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)

                    board.controller_00.origin()
                    pub_to_dqn.target_00.publish(board.controller_00.sp)     
                    
                    sub_from_dqn.reset = False
                    # finally pending the action py code until initial states of drones
                    while (not rospy.is_shutdown()):
                        if (board.controller_00.local_pos.x <= 0.1 and board.controller_00.local_pos.x >= -0.1) and \
                            (board.controller_00.local_pos.y <= 0.1 and board.controller_00.local_pos.y >= -0.1) and \
                            (board.controller_00.local_pos.z <= 3.1 and board.controller_00.local_pos.z >= 2.9):
                            pub_to_dqn.pub_reset_00.publish(data=False)
                            break
                    break
                    
            # negative x - Check OK
            if sub_from_dqn.action_index == 2:
                print("select the negative x-axis")
                board.controller_00.neg_x_dir()
                pub_to_dqn.target_00.publish(board.controller_00.sp)
                while (not rospy.is_shutdown()):
                   
                    if (abs(board.controller_00.sp.position.x - board.controller_00.local_pos.x) <= 0.2 and \
                        sub_from_dqn.reset == False):
                        break
                    if np.amin(sub_from_dqn.laser_array) < 3:
                        board.controller_00.stop()
                        pub_to_dqn.target_00.publish(board.controller_00.sp)
                        pub_to_dqn.pub_reset_00.publish(data=True)
                        sub_from_dqn.reset = True   
                        break
                    if -1.0 >= board.controller_00.sp.position.z or board.controller_00.sp.position.z >= 80.0:
                        board.controller_00.stop()
                        pub_to_dqn.target_00.publish(board.controller_00.sp)
                        pub_to_dqn.pub_reset_00.publish(data=True)
                        sub_from_dqn.reset = True
                        break

                    continue
                
                if sub_from_dqn.reset == False:
                
                # 4. one time, publish the reward to DQN code
                # pub_to_dqn.pub_reward.publish(pub_to_dqn.reward_msg)
                    #pub_to_dqn.reward.publish(data=2.0)
                    pub_to_dqn.reward_msg.data = 2.0
                    pub_to_dqn.DQN_reward.publish(pub_to_dqn.reward_msg)
                    rospy.sleep(0.1)
                # 5. one time, publish the done message to DQN code
                    pub_to_dqn.done.publish(data=False)
                
                    pub_to_dqn.pending_msg = 1
                # 6. publish the pending signal message to DQN code
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)
                        
                    rospy.sleep(0.1)
                    pub_to_dqn.pending_msg = 0
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)                                                                      
                    break
                
                else:
                    # When simulater is restarted because of the obstacles, the agents gain negative rewards 
                    pub_to_dqn.reward_msg.data = -100.0
                    pub_to_dqn.DQN_reward.publish(pub_to_dqn.reward_msg)
                    rospy.sleep(0.1)
                    
                    # Then action py publish the done message to DQN code
                    pub_to_dqn.done.publish(data=True)
                    
                    pub_to_dqn.pending_msg = 1
                    # publish the pending signal message to DQN code
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)
                        
                    rospy.sleep(0.1)
                    pub_to_dqn.pending_msg = 0
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)

                    board.controller_00.origin()
                    pub_to_dqn.target_00.publish(board.controller_00.sp)      
                    
                    sub_from_dqn.reset = False
                    # finally pending the action py code until initial states of drones
                    while (not rospy.is_shutdown()):
                         if (board.controller_00.local_pos.x <= 0.1 and board.controller_00.local_pos.x >= -0.1) and \
                            (board.controller_00.local_pos.y <= 0.1 and board.controller_00.local_pos.y >= -0.1) and \
                            (board.controller_00.local_pos.z <= 3.1 and board.controller_00.local_pos.z >= 2.9):
                            pub_to_dqn.pub_reset_00.publish(data=False)
                            break
                    break
            
            # negative y - Check OK
            if sub_from_dqn.action_index == 3:
                print("select the negative y-axis")
                board.controller_00.neg_y_dir()
                pub_to_dqn.target_00.publish(board.controller_00.sp)
                
                while (not rospy.is_shutdown()):
                    
                    if (abs(board.controller_00.sp.position.y - board.controller_00.local_pos.y) <= 0.2 and \
                        sub_from_dqn.reset == False):
                        break
                    if np.amin(sub_from_dqn.laser_array) < 3:
                        board.controller_00.stop()
                        pub_to_dqn.target_00.publish(board.controller_00.sp)
                        pub_to_dqn.pub_reset_00.publish(data=True)
                        sub_from_dqn.reset = True   
                        break
                    if -1.0 >= board.controller_00.sp.position.z or board.controller_00.sp.position.z >= 80.0:
                        board.controller_00.stop()
                        pub_to_dqn.target_00.publish(board.controller_00.sp)
                        pub_to_dqn.pub_reset_00.publish(data=True)
                        sub_from_dqn.reset = True
                        break

                    continue
                        
                if sub_from_dqn.reset == False:
                
                # 4. one time, publish the reward to DQN code
                    pub_to_dqn.reward_msg.data = 3.0
                    pub_to_dqn.DQN_reward.publish(pub_to_dqn.reward_msg)
                    rospy.sleep(0.1)
                # 5. one time, publish the done message to DQN code
                    pub_to_dqn.done.publish(data=False)
                
                    pub_to_dqn.pending_msg = 1
                # 6. publish the pending signal message to DQN code
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)
                        
                    rospy.sleep(0.1)
                    pub_to_dqn.pending_msg = 0
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)                                                                  
                    break        
                else:
                    # When simulater is restarted because of the obstacles, the agents gain negative rewards 
                    pub_to_dqn.reward_msg.data = -100.0
                    pub_to_dqn.DQN_reward.publish(pub_to_dqn.reward_msg)
                    rospy.sleep(0.1)
                    
                    # Then action py publish the done message to DQN code
                    pub_to_dqn.done.publish(data=True)
                    
                    pub_to_dqn.pending_msg = 1
                    # publish the pending signal message to DQN code
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)
                        
                    rospy.sleep(0.1)
                    pub_to_dqn.pending_msg = 0
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)

                    board.controller_00.origin()
                    pub_to_dqn.target_00.publish(board.controller_00.sp)   
                    
                    sub_from_dqn.reset = False
                    # finally pending the action py code until initial states of drones
                    while (not rospy.is_shutdown()):
                        if (board.controller_00.local_pos.x <= 0.1 and board.controller_00.local_pos.x >= -0.1) and \
                            (board.controller_00.local_pos.y <= 0.1 and board.controller_00.local_pos.y >= -0.1) and \
                            (board.controller_00.local_pos.z <= 3.1 and board.controller_00.local_pos.z >= 2.9):
                            pub_to_dqn.pub_reset_00.publish(data=False)
                            break
                    break

            # positive x + positive y - Check OK
            if sub_from_dqn.action_index == 4:
                print("select the x-axis + y-axis")
                board.controller_00.action_4()
                pub_to_dqn.target_00.publish(board.controller_00.sp)

                while (not rospy.is_shutdown()):
                    
                    if (abs(board.controller_00.sp.position.x - board.controller_00.local_pos.x) <= 0.2 and \
                        abs(board.controller_00.sp.position.y - board.controller_00.local_pos.y) <= 0.2 and \
                            sub_from_dqn.reset == False):
                        break
                    if np.amin(sub_from_dqn.laser_array) < 3:
                        board.controller_00.stop()
                        pub_to_dqn.target_00.publish(board.controller_00.sp)
                        pub_to_dqn.pub_reset_00.publish(data=True)
                        sub_from_dqn.reset = True
                        break
                    if -1.0 >= board.controller_00.sp.position.z or board.controller_00.sp.position.z >= 80.0:
                        board.controller_00.stop()
                        pub_to_dqn.target_00.publish(board.controller_00.sp)
                        pub_to_dqn.pub_reset_00.publish(data=True)
                        sub_from_dqn.reset = True
                        break

                    continue
                if sub_from_dqn.reset == False:
                
                # 4. one time, publish the reward to DQN code
                    pub_to_dqn.reward_msg.data = 0.0
                    pub_to_dqn.DQN_reward.publish(pub_to_dqn.reward_msg)
                    rospy.sleep(0.1)
                # 5. one time, publish the done message to DQN code
                    pub_to_dqn.done.publish(data=False)
                    #if current position is terminal, action node is publishing the true signal
                
                    pub_to_dqn.pending_msg = 1
                # 6. publish the pending signal message to DQN code
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)
                        
                    rospy.sleep(0.1)
                    pub_to_dqn.pending_msg = 0
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)                                                                        
                    break
                
                else:
                    # When simulater is restarted because of the obstacles, the agents gain negative rewards 
                    pub_to_dqn.reward_msg.data = -100.0
                    pub_to_dqn.DQN_reward.publish(pub_to_dqn.reward_msg)
                    rospy.sleep(0.1)
                    
                    # Then action py publish the done message to DQN code
                    pub_to_dqn.done.publish(data=True)
                    
                    pub_to_dqn.pending_msg = 1
                    # publish the pending signal message to DQN code
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)
                        
                    rospy.sleep(0.1)
                    pub_to_dqn.pending_msg = 0
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)

                    board.controller_00.origin()
                    pub_to_dqn.target_00.publish(board.controller_00.sp)  
                    
                    sub_from_dqn.reset = False
                    # finally pending the action py code until initial states of drones
                    while (not rospy.is_shutdown()):
                        if (board.controller_00.local_pos.x <= 0.1 and board.controller_00.local_pos.x >= -0.1) and \
                            (board.controller_00.local_pos.y <= 0.1 and board.controller_00.local_pos.y >= -0.1) and \
                            (board.controller_00.local_pos.z <= 3.1 and board.controller_00.local_pos.z >= 2.9):
                            pub_to_dqn.pub_reset_00.publish(data=False)
                            break
                    break
                    
            # positive x + negative y - Check OK
            if sub_from_dqn.action_index == 5:
                print("select the x-axis and negative y-axis")
                board.controller_00.action_5()
                pub_to_dqn.target_00.publish(board.controller_00.sp)
                        
                while (not rospy.is_shutdown()): 
                    
                    if (abs(board.controller_00.sp.position.x - board.controller_00.local_pos.x) <= 0.2 and \
                        abs(board.controller_00.sp.position.y - board.controller_00.local_pos.y) <= 0.2 and \
                            sub_from_dqn.reset == False):
                        break
                    if np.amin(sub_from_dqn.laser_array) < 3:
                        board.controller_00.stop()
                        pub_to_dqn.target_00.publish(board.controller_00.sp)
                        pub_to_dqn.pub_reset_00.publish(data=True)
                        sub_from_dqn.reset = True               
                        break
                    if -1.0 >= board.controller_00.sp.position.z or board.controller_00.sp.position.z >= 80.0:
                        board.controller_00.stop()
                        pub_to_dqn.target_00.publish(board.controller_00.sp)
                        pub_to_dqn.pub_reset_00.publish(data=True)
                        sub_from_dqn.reset = True
                        break

                    continue
                
                if sub_from_dqn.reset == False:
                
                # 4. one time, publish the reward to DQN code
                # pub_to_dqn.pub_reward.publish(pub_to_dqn.reward_msg)
                    pub_to_dqn.reward_msg.data = 1.0
                    pub_to_dqn.DQN_reward.publish(pub_to_dqn.reward_msg)
                    #pub_to_dqn.reward.publish(data=1.0)
                # 5. one time, publish the done message to DQN code
                    pub_to_dqn.done.publish(data=False)
                
                    pub_to_dqn.pending_msg = 1
                # 6. publish the pending signal message to DQN code
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)
                    #if current position is terminal, action node is publishing the true signal
                        
                    rospy.sleep(0.1)
                    pub_to_dqn.pending_msg = 0
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)                                                                   
                    break
                
                else:
                    # When simulater is restarted because of the obstacles, the agents gain negative rewards 
                    pub_to_dqn.reward_msg.data = -100.0
                    pub_to_dqn.DQN_reward.publish(pub_to_dqn.reward_msg)
                    rospy.sleep(0.1)
                    
                    # Then action py publish the done message to DQN code
                    pub_to_dqn.done.publish(data=True)
                    
                    pub_to_dqn.pending_msg = 1
                    # publish the pending signal message to DQN code
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)
                        
                    rospy.sleep(0.1)
                    pub_to_dqn.pending_msg = 0
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)

                    board.controller_00.origin()
                    pub_to_dqn.target_00.publish(board.controller_00.sp)     
                    
                    sub_from_dqn.reset = False
                    # finally pending the action py code until initial states of drones
                    while (not rospy.is_shutdown()):
                        if (board.controller_00.local_pos.x <= 0.1 and board.controller_00.local_pos.x >= -0.1) and \
                            (board.controller_00.local_pos.y <= 0.1 and board.controller_00.local_pos.y >= -0.1) and \
                            (board.controller_00.local_pos.z <= 3.1 and board.controller_00.local_pos.z >= 2.9):
                            pub_to_dqn.pub_reset_00.publish(data=False)
                            break
                    break
                    
            # negative x + positive y - Check OK
            if sub_from_dqn.action_index == 6:
                print("select the negative x-axis and y-axis")
                board.controller_00.action_6()
                pub_to_dqn.target_00.publish(board.controller_00.sp)
                
                while (not rospy.is_shutdown()):
                    
                    if (abs(board.controller_00.sp.position.x - board.controller_00.local_pos.x) <= 0.2 and \
                        abs(board.controller_00.sp.position.y - board.controller_00.local_pos.y) <= 0.2 and \
                            sub_from_dqn.reset == False):
                        break
                    if np.amin(sub_from_dqn.laser_array) < 3:
                        board.controller_00.stop()
                        pub_to_dqn.target_00.publish(board.controller_00.sp)
                        pub_to_dqn.pub_reset_00.publish(data=True)
                        sub_from_dqn.reset = True   
                        break
                    if -1.0 >= board.controller_00.sp.position.z or board.controller_00.sp.position.z >= 80.0:
                        board.controller_00.stop()
                        pub_to_dqn.target_00.publish(board.controller_00.sp)
                        pub_to_dqn.pub_reset_00.publish(data=True)
                        sub_from_dqn.reset = True
                        break

                    continue
                
                if sub_from_dqn.reset == False:              
                # 4. one time, publish the reward to DQN code
                # pub_to_dqn.pub_reward.publish(pub_to_dqn.reward_msg)
                    #pub_to_dqn.reward.publish(data=2.0)
                    pub_to_dqn.reward_msg.data = 2.0
                    pub_to_dqn.DQN_reward.publish(pub_to_dqn.reward_msg)
                    rospy.sleep(0.1)
                # 5. one time, publish the done message to DQN code
                    pub_to_dqn.done.publish(data=False)
                
                    pub_to_dqn.pending_msg = 1
                # 6. publish the pending signal message to DQN code
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)
                        
                    rospy.sleep(0.1)
                    pub_to_dqn.pending_msg = 0
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)                                                                      
                    break
                
                else:
                    # When simulater is restarted because of the obstacles, the agents gain negative rewards 
                    pub_to_dqn.reward_msg.data = -100.0
                    pub_to_dqn.DQN_reward.publish(pub_to_dqn.reward_msg)
                    rospy.sleep(0.1)
                    
                    # Then action py publish the done message to DQN code
                    pub_to_dqn.done.publish(data=True)
                    
                    pub_to_dqn.pending_msg = 1
                    # publish the pending signal message to DQN code
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)
                        
                    rospy.sleep(0.1)
                    pub_to_dqn.pending_msg = 0
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)

                    board.controller_00.origin()
                    pub_to_dqn.target_00.publish(board.controller_00.sp)      
                    
                    sub_from_dqn.reset = False
                    # finally pending the action py code until initial states of drones
                    while (not rospy.is_shutdown()):
                         if (board.controller_00.local_pos.x <= 0.1 and board.controller_00.local_pos.x >= -0.1) and \
                            (board.controller_00.local_pos.y <= 0.1 and board.controller_00.local_pos.y >= -0.1) and \
                            (board.controller_00.local_pos.z <= 3.1 and board.controller_00.local_pos.z >= 2.9):
                            pub_to_dqn.pub_reset_00.publish(data=False)
                            break
                    break
            
            # negative x + negative y - Check OK
            if sub_from_dqn.action_index == 7:
                print("select the negative x-axis and negative y-axis")
                board.controller_00.action_7()
                pub_to_dqn.target_00.publish(board.controller_00.sp)
                
                while (not rospy.is_shutdown()):
                    
                    if (abs(board.controller_00.sp.position.x - board.controller_00.local_pos.x) <= 0.2 and \
                        abs(board.controller_00.sp.position.y - board.controller_00.local_pos.y) <= 0.2 and \
                            sub_from_dqn.reset == False):
                        break
                    if np.amin(sub_from_dqn.laser_array) < 3:
                        board.controller_00.stop()
                        pub_to_dqn.target_00.publish(board.controller_00.sp)
                        pub_to_dqn.pub_reset_00.publish(data=True)
                        sub_from_dqn.reset = True   
                        break
                    if -1.0 >= board.controller_00.sp.position.z or board.controller_00.sp.position.z >= 80.0:
                        board.controller_00.stop()
                        pub_to_dqn.target_00.publish(board.controller_00.sp)
                        pub_to_dqn.pub_reset_00.publish(data=True)
                        sub_from_dqn.reset = True
                        break
                    
                    continue
                        
                if sub_from_dqn.reset == False:              
                # 4. one time, publish the reward to DQN code
                    pub_to_dqn.reward_msg.data = 3.0
                    pub_to_dqn.DQN_reward.publish(pub_to_dqn.reward_msg)
                    rospy.sleep(0.1)
                # 5. one time, publish the done message to DQN code
                    pub_to_dqn.done.publish(data=False)
                
                    pub_to_dqn.pending_msg = 1
                # 6. publish the pending signal message to DQN code
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)
                        
                    rospy.sleep(0.1)
                    pub_to_dqn.pending_msg = 0
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)                                                                  
                    break        
                else:
                    # When simulater is restarted because of the obstacles, the agents gain negative rewards 
                    pub_to_dqn.reward_msg.data = -100.0
                    pub_to_dqn.DQN_reward.publish(pub_to_dqn.reward_msg)
                    rospy.sleep(0.1)
                    
                    # Then action py publish the done message to DQN code
                    pub_to_dqn.done.publish(data=True)
                    
                    pub_to_dqn.pending_msg = 1
                    # publish the pending signal message to DQN code
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)
                        
                    rospy.sleep(0.1)
                    pub_to_dqn.pending_msg = 0
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)

                    board.controller_00.origin()
                    pub_to_dqn.target_00.publish(board.controller_00.sp)   
                    
                    sub_from_dqn.reset = False
                    # finally pending the action py code until initial states of drones
                    while (not rospy.is_shutdown()):
                        if (board.controller_00.local_pos.x <= 0.1 and board.controller_00.local_pos.x >= -0.1) and \
                            (board.controller_00.local_pos.y <= 0.1 and board.controller_00.local_pos.y >= -0.1) and \
                            (board.controller_00.local_pos.z <= 3.1 and board.controller_00.local_pos.z >= 2.9):
                            pub_to_dqn.pub_reset_00.publish(data=False)
                            break
                    break

            # up - Check OK
            if sub_from_dqn.action_index == 8:
                print("select the up")
                board.controller_00.up()
                pub_to_dqn.target_00.publish(board.controller_00.sp)
                
                while (not rospy.is_shutdown()):
                    
                    if (abs(board.controller_00.sp.position.z - board.controller_00.local_pos.z) <= 0.2 and \
                        sub_from_dqn.reset == False):
                        break
                    if np.amin(sub_from_dqn.laser_array) < 3:
                        board.controller_00.stop()
                        pub_to_dqn.target_00.publish(board.controller_00.sp)
                        pub_to_dqn.pub_reset_00.publish(data=True)
                        sub_from_dqn.reset = True
                        break
                    if -1.0 >= board.controller_00.sp.position.z or board.controller_00.sp.position.z >= 80.0:
                        board.controller_00.stop()
                        pub_to_dqn.target_00.publish(board.controller_00.sp)
                        pub_to_dqn.pub_reset_00.publish(data=True)
                        sub_from_dqn.reset = True
                        break

                    continue

                if sub_from_dqn.reset == False:
                # 4. one time, publish the reward to DQN code
                    pub_to_dqn.reward_msg.data = 0.0
                    pub_to_dqn.DQN_reward.publish(pub_to_dqn.reward_msg)
                    rospy.sleep(0.1)
                # 5. one time, publish the done message to DQN code
                    pub_to_dqn.done.publish(data=False)
                    #if current position is terminal, action node is publishing the true signal
                
                    pub_to_dqn.pending_msg = 1
                # 6. publish the pending signal message to DQN code
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)
                        
                    rospy.sleep(0.1)
                    pub_to_dqn.pending_msg = 0
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)                                                                        
                    break
                
                else:
                    # When simulater is restarted because of the obstacles, the agents gain negative rewards 
                    pub_to_dqn.reward_msg.data = -100.0
                    pub_to_dqn.DQN_reward.publish(pub_to_dqn.reward_msg)
                    rospy.sleep(0.1)
                    
                    # Then action py publish the done message to DQN code
                    pub_to_dqn.done.publish(data=True)
                    
                    pub_to_dqn.pending_msg = 1
                    # publish the pending signal message to DQN code
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)
                        
                    rospy.sleep(0.1)
                    pub_to_dqn.pending_msg = 0
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)

                    board.controller_00.origin()
                    pub_to_dqn.target_00.publish(board.controller_00.sp)  
                    
                    sub_from_dqn.reset = False
                    # finally pending the action py code until initial states of drones
                    while (not rospy.is_shutdown()):
                        if (board.controller_00.local_pos.x <= 0.1 and board.controller_00.local_pos.x >= -0.1) and \
                            (board.controller_00.local_pos.y <= 0.1 and board.controller_00.local_pos.y >= -0.1) and \
                            (board.controller_00.local_pos.z <= 3.1 and board.controller_00.local_pos.z >= 2.9):
                            pub_to_dqn.pub_reset_00.publish(data=False)
                            break
                    break
            
            # down - Check OK
            if sub_from_dqn.action_index == 9:
                print("select the down")
                board.controller_00.down()
                pub_to_dqn.target_00.publish(board.controller_00.sp)
                
                while (not rospy.is_shutdown()):
                    
                    if (abs(board.controller_00.sp.position.z - board.controller_00.local_pos.z) <= 0.2 and \
                        sub_from_dqn.reset == False):
                        break
                    if np.amin(sub_from_dqn.laser_array) < 3:
                        board.controller_00.stop()
                        pub_to_dqn.target_00.publish(board.controller_00.sp)
                        pub_to_dqn.pub_reset_00.publish(data=True)
                        sub_from_dqn.reset = True
                        break
                    if -1.0 >= board.controller_00.sp.position.z or board.controller_00.sp.position.z >= 80.0:
                        board.controller_00.stop()
                        pub_to_dqn.target_00.publish(board.controller_00.sp)
                        pub_to_dqn.pub_reset_00.publish(data=True)
                        sub_from_dqn.reset = True
                        break

                    continue
                
                if sub_from_dqn.reset == False:
                
                # 4. one time, publish the reward to DQN code
                    pub_to_dqn.reward_msg.data = 0.0
                    pub_to_dqn.DQN_reward.publish(pub_to_dqn.reward_msg)
                    rospy.sleep(0.1)
                # 5. one time, publish the done message to DQN code
                    pub_to_dqn.done.publish(data=False)
                    #if current position is terminal, action node is publishing the true signal
                
                    pub_to_dqn.pending_msg = 1
                # 6. publish the pending signal message to DQN code
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)
                        
                    rospy.sleep(0.1)
                    pub_to_dqn.pending_msg = 0
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)                                                                        
                    break
                
                else:
                    # When simulater is restarted because of the obstacles, the agents gain negative rewards 
                    pub_to_dqn.reward_msg.data = -100.0
                    pub_to_dqn.DQN_reward.publish(pub_to_dqn.reward_msg)
                    rospy.sleep(0.1)
                    
                    # Then action py publish the done message to DQN code
                    pub_to_dqn.done.publish(data=True)
                    
                    pub_to_dqn.pending_msg = 1
                    # publish the pending signal message to DQN code
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)
                        
                    rospy.sleep(0.1)
                    pub_to_dqn.pending_msg = 0
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)

                    board.controller_00.origin()
                    pub_to_dqn.target_00.publish(board.controller_00.sp)  
                    
                    sub_from_dqn.reset = False
                    # finally pending the action py code until initial states of drones
                    while (not rospy.is_shutdown()):
                        if (board.controller_00.local_pos.x <= 0.1 and board.controller_00.local_pos.x >= -0.1) and \
                            (board.controller_00.local_pos.y <= 0.1 and board.controller_00.local_pos.y >= -0.1) and \
                            (board.controller_00.local_pos.z <= 3.1 and board.controller_00.local_pos.z >= 2.9):
                            pub_to_dqn.pub_reset_00.publish(data=False)
                            break
                    break

            # up + positive x - Check OK
            if sub_from_dqn.action_index == 10:
                print("select the up + positive x-axis")
                board.controller_00.action_10()
                pub_to_dqn.target_00.publish(board.controller_00.sp)

                while (not rospy.is_shutdown()):
                    
                    if (abs(board.controller_00.sp.position.x - board.controller_00.local_pos.x) <= 0.2 and \
                        abs(board.controller_00.sp.position.z - board.controller_00.local_pos.z) <= 0.2 and \
                            sub_from_dqn.reset == False):
                        break
                    if np.amin(sub_from_dqn.laser_array) < 3:
                        board.controller_00.stop()
                        pub_to_dqn.target_00.publish(board.controller_00.sp)
                        pub_to_dqn.pub_reset_00.publish(data=True)
                        sub_from_dqn.reset = True
                        break
                    if -1.0 >= board.controller_00.sp.position.z or board.controller_00.sp.position.z >= 80.0:
                        board.controller_00.stop()
                        pub_to_dqn.target_00.publish(board.controller_00.sp)
                        pub_to_dqn.pub_reset_00.publish(data=True)
                        sub_from_dqn.reset = True
                        break

                    continue

                if sub_from_dqn.reset == False:
                
                # 4. one time, publish the reward to DQN code
                    pub_to_dqn.reward_msg.data = 0.0
                    pub_to_dqn.DQN_reward.publish(pub_to_dqn.reward_msg)
                    rospy.sleep(0.1)
                # 5. one time, publish the done message to DQN code
                    pub_to_dqn.done.publish(data=False)
                    #if current position is terminal, action node is publishing the true signal
                
                    pub_to_dqn.pending_msg = 1
                # 6. publish the pending signal message to DQN code
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)
                        
                    rospy.sleep(0.1)
                    pub_to_dqn.pending_msg = 0
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)                                                                        
                    break
                
                else:
                    # When simulater is restarted because of the obstacles, the agents gain negative rewards 
                    pub_to_dqn.reward_msg.data = -100.0
                    pub_to_dqn.DQN_reward.publish(pub_to_dqn.reward_msg)
                    rospy.sleep(0.1)
                    
                    # Then action py publish the done message to DQN code
                    pub_to_dqn.done.publish(data=True)
                    
                    pub_to_dqn.pending_msg = 1
                    # publish the pending signal message to DQN code
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)
                        
                    rospy.sleep(0.1)
                    pub_to_dqn.pending_msg = 0
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)

                    board.controller_00.origin()
                    pub_to_dqn.target_00.publish(board.controller_00.sp)  
                    
                    sub_from_dqn.reset = False
                    # finally pending the action py code until initial states of drones
                    while (not rospy.is_shutdown()):
                        if (board.controller_00.local_pos.x <= 0.1 and board.controller_00.local_pos.x >= -0.1) and \
                            (board.controller_00.local_pos.y <= 0.1 and board.controller_00.local_pos.y >= -0.1) and \
                            (board.controller_00.local_pos.z <= 3.1 and board.controller_00.local_pos.z >= 2.9):
                            pub_to_dqn.pub_reset_00.publish(data=False)
                            break
                    break
            
                        # up-positive x
            
            # up + positive y - Check OK
            if sub_from_dqn.action_index == 11:
                print("select the up + positive y-axis")
                board.controller_00.action_11()
                pub_to_dqn.target_00.publish(board.controller_00.sp)

                while (not rospy.is_shutdown()):
                    
                    if (abs(board.controller_00.sp.position.y - board.controller_00.local_pos.y) <= 0.2 and \
                        abs(board.controller_00.sp.position.z - board.controller_00.local_pos.z) <= 0.2 and \
                            sub_from_dqn.reset == False):
                        break
                    if np.amin(sub_from_dqn.laser_array) < 3:
                        board.controller_00.stop()
                        pub_to_dqn.target_00.publish(board.controller_00.sp)
                        pub_to_dqn.pub_reset_00.publish(data=True)
                        sub_from_dqn.reset = True
                        break
                    if -1.0 >= board.controller_00.sp.position.z or board.controller_00.sp.position.z >= 80.0:
                        board.controller_00.stop()
                        pub_to_dqn.target_00.publish(board.controller_00.sp)
                        pub_to_dqn.pub_reset_00.publish(data=True)
                        sub_from_dqn.reset = True
                        break

                    continue

                if sub_from_dqn.reset == False:
                
                # 4. one time, publish the reward to DQN code
                    pub_to_dqn.reward_msg.data = 0.0
                    pub_to_dqn.DQN_reward.publish(pub_to_dqn.reward_msg)
                    rospy.sleep(0.1)
                # 5. one time, publish the done message to DQN code
                    pub_to_dqn.done.publish(data=False)
                    #if current position is terminal, action node is publishing the true signal
                
                    pub_to_dqn.pending_msg = 1
                # 6. publish the pending signal message to DQN code
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)
                        
                    rospy.sleep(0.1)
                    pub_to_dqn.pending_msg = 0
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)                                                                        
                    break
                
                else:
                    # When simulater is restarted because of the obstacles, the agents gain negative rewards 
                    pub_to_dqn.reward_msg.data = -100.0
                    pub_to_dqn.DQN_reward.publish(pub_to_dqn.reward_msg)
                    rospy.sleep(0.1)
                    
                    # Then action py publish the done message to DQN code
                    pub_to_dqn.done.publish(data=True)
                    
                    pub_to_dqn.pending_msg = 1
                    # publish the pending signal message to DQN code
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)
                        
                    rospy.sleep(0.1)
                    pub_to_dqn.pending_msg = 0
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)

                    board.controller_00.origin()
                    pub_to_dqn.target_00.publish(board.controller_00.sp)  
                    
                    sub_from_dqn.reset = False
                    # finally pending the action py code until initial states of drones
                    while (not rospy.is_shutdown()):
                        if (board.controller_00.local_pos.x <= 0.1 and board.controller_00.local_pos.x >= -0.1) and \
                            (board.controller_00.local_pos.y <= 0.1 and board.controller_00.local_pos.y >= -0.1) and \
                            (board.controller_00.local_pos.z <= 3.1 and board.controller_00.local_pos.z >= 2.9):
                            pub_to_dqn.pub_reset_00.publish(data=False)
                            break
                    break
            
            # up + negative x - Check OK
            if sub_from_dqn.action_index == 12:
                print("select the up + negative x-axis")
                board.controller_00.action_12()
                pub_to_dqn.target_00.publish(board.controller_00.sp)

                while (not rospy.is_shutdown()):
                    
                    if (abs(board.controller_00.sp.position.x - board.controller_00.local_pos.x) <= 0.2 and \
                        abs(board.controller_00.sp.position.z - board.controller_00.local_pos.z) <= 0.2 and \
                            sub_from_dqn.reset == False):
                        break
                    if np.amin(sub_from_dqn.laser_array) < 3:
                        board.controller_00.stop()
                        pub_to_dqn.target_00.publish(board.controller_00.sp)
                        pub_to_dqn.pub_reset_00.publish(data=True)
                        sub_from_dqn.reset = True
                        break
                    if -1.0 >= board.controller_00.sp.position.z or board.controller_00.sp.position.z >= 80.0:
                        board.controller_00.stop()
                        pub_to_dqn.target_00.publish(board.controller_00.sp)
                        pub_to_dqn.pub_reset_00.publish(data=True)
                        sub_from_dqn.reset = True
                        break
                    
                    continue

                if sub_from_dqn.reset == False:
                
                # 4. one time, publish the reward to DQN code
                    pub_to_dqn.reward_msg.data = 0.0
                    pub_to_dqn.DQN_reward.publish(pub_to_dqn.reward_msg)
                    rospy.sleep(0.1)
                # 5. one time, publish the done message to DQN code
                    pub_to_dqn.done.publish(data=False)
                    #if current position is terminal, action node is publishing the true signal
                
                    pub_to_dqn.pending_msg = 1
                # 6. publish the pending signal message to DQN code
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)
                        
                    rospy.sleep(0.1)
                    pub_to_dqn.pending_msg = 0
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)                                                                        
                    break
                
                else:
                    # When simulater is restarted because of the obstacles, the agents gain negative rewards 
                    pub_to_dqn.reward_msg.data = -100.0
                    pub_to_dqn.DQN_reward.publish(pub_to_dqn.reward_msg)
                    rospy.sleep(0.1)
                    
                    # Then action py publish the done message to DQN code
                    pub_to_dqn.done.publish(data=True)
                    
                    pub_to_dqn.pending_msg = 1
                    # publish the pending signal message to DQN code
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)
                        
                    rospy.sleep(0.1)
                    pub_to_dqn.pending_msg = 0
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)

                    board.controller_00.origin()
                    pub_to_dqn.target_00.publish(board.controller_00.sp)  
                    
                    sub_from_dqn.reset = False
                    # finally pending the action py code until initial states of drones
                    while (not rospy.is_shutdown()):
                        if (board.controller_00.local_pos.x <= 0.1 and board.controller_00.local_pos.x >= -0.1) and \
                            (board.controller_00.local_pos.y <= 0.1 and board.controller_00.local_pos.y >= -0.1) and \
                            (board.controller_00.local_pos.z <= 3.1 and board.controller_00.local_pos.z >= 2.9):
                            pub_to_dqn.pub_reset_00.publish(data=False)
                            break
                    break            
            
            # up + negative y - Check OK
            if sub_from_dqn.action_index == 13:
                print("select the up + negative y-axis")
                board.controller_00.action_13()
                pub_to_dqn.target_00.publish(board.controller_00.sp)

                while (not rospy.is_shutdown()):
                    
                    if (abs(board.controller_00.sp.position.y - board.controller_00.local_pos.y) <= 0.2 and \
                        abs(board.controller_00.sp.position.z - board.controller_00.local_pos.z) <= 0.2 and \
                            sub_from_dqn.reset == False):
                        break
                    if np.amin(sub_from_dqn.laser_array) < 3:
                        board.controller_00.stop()
                        pub_to_dqn.target_00.publish(board.controller_00.sp)
                        pub_to_dqn.pub_reset_00.publish(data=True)
                        sub_from_dqn.reset = True
                        break
                    if -1.0 >= board.controller_00.sp.position.z or board.controller_00.sp.position.z >= 80.0:
                        board.controller_00.stop()
                        pub_to_dqn.target_00.publish(board.controller_00.sp)
                        pub_to_dqn.pub_reset_00.publish(data=True)
                        sub_from_dqn.reset = True
                        break
                    
                    continue

                if sub_from_dqn.reset == False:
                
                # 4. one time, publish the reward to DQN code
                    pub_to_dqn.reward_msg.data = 0.0
                    pub_to_dqn.DQN_reward.publish(pub_to_dqn.reward_msg)
                    rospy.sleep(0.1)
                # 5. one time, publish the done message to DQN code
                    pub_to_dqn.done.publish(data=False)
                    #if current position is terminal, action node is publishing the true signal
                
                    pub_to_dqn.pending_msg = 1
                # 6. publish the pending signal message to DQN code
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)
                        
                    rospy.sleep(0.1)
                    pub_to_dqn.pending_msg = 0
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)                                                                        
                    break
                
                else:
                    # When simulater is restarted because of the obstacles, the agents gain negative rewards 
                    pub_to_dqn.reward_msg.data = -100.0
                    pub_to_dqn.DQN_reward.publish(pub_to_dqn.reward_msg)
                    rospy.sleep(0.1)
                    
                    # Then action py publish the done message to DQN code
                    pub_to_dqn.done.publish(data=True)
                    
                    pub_to_dqn.pending_msg = 1
                    # publish the pending signal message to DQN code
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)
                        
                    rospy.sleep(0.1)
                    pub_to_dqn.pending_msg = 0
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)

                    board.controller_00.origin()
                    pub_to_dqn.target_00.publish(board.controller_00.sp)  
                    
                    sub_from_dqn.reset = False
                    # finally pending the action py code until initial states of drones
                    while (not rospy.is_shutdown()):
                        if (board.controller_00.local_pos.x <= 0.1 and board.controller_00.local_pos.x >= -0.1) and \
                            (board.controller_00.local_pos.y <= 0.1 and board.controller_00.local_pos.y >= -0.1) and \
                            (board.controller_00.local_pos.z <= 3.1 and board.controller_00.local_pos.z >= 2.9):
                            pub_to_dqn.pub_reset_00.publish(data=False)
                            break
                    break
 
            # up + positive x + positive y - Check OK
            if sub_from_dqn.action_index == 14:
                print("select the up + x-axis + y-axis")
                board.controller_00.action_14()
                pub_to_dqn.target_00.publish(board.controller_00.sp)

                while (not rospy.is_shutdown()):
                    
                    if (abs(board.controller_00.sp.position.x - board.controller_00.local_pos.x) <= 0.2 and \
                        abs(board.controller_00.sp.position.y - board.controller_00.local_pos.y) <= 0.2 and \
                            abs(board.controller_00.sp.position.z - board.controller_00.local_pos.z) <= 0.2 and \
                                sub_from_dqn.reset == False):
                        break
                    if np.amin(sub_from_dqn.laser_array) < 3:
                        board.controller_00.stop()
                        pub_to_dqn.target_00.publish(board.controller_00.sp)
                        pub_to_dqn.pub_reset_00.publish(data=True)
                        sub_from_dqn.reset = True
                        break                    
                    if -1.0 >= board.controller_00.sp.position.z or board.controller_00.sp.position.z >= 80.0:
                        board.controller_00.stop()
                        pub_to_dqn.target_00.publish(board.controller_00.sp)
                        pub_to_dqn.pub_reset_00.publish(data=True)
                        sub_from_dqn.reset = True
                        break
                    
                    continue

                if sub_from_dqn.reset == False:               
                # 4. one time, publish the reward to DQN code
                    pub_to_dqn.reward_msg.data = 0.0
                    pub_to_dqn.DQN_reward.publish(pub_to_dqn.reward_msg)
                    rospy.sleep(0.1)
                # 5. one time, publish the done message to DQN code
                    pub_to_dqn.done.publish(data=False)
                    #if current position is terminal, action node is publishing the true signal
                
                    pub_to_dqn.pending_msg = 1
                # 6. publish the pending signal message to DQN code
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)
                        
                    rospy.sleep(0.1)
                    pub_to_dqn.pending_msg = 0
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)                                                                        
                    break
                
                else:
                    # When simulater is restarted because of the obstacles, the agents gain negative rewards 
                    pub_to_dqn.reward_msg.data = -100.0
                    pub_to_dqn.DQN_reward.publish(pub_to_dqn.reward_msg)
                    rospy.sleep(0.1)
                    
                    # Then action py publish the done message to DQN code
                    pub_to_dqn.done.publish(data=True)
                    
                    pub_to_dqn.pending_msg = 1
                    # publish the pending signal message to DQN code
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)
                        
                    rospy.sleep(0.1)
                    pub_to_dqn.pending_msg = 0
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)

                    board.controller_00.origin()
                    pub_to_dqn.target_00.publish(board.controller_00.sp)  
                    
                    sub_from_dqn.reset = False
                    # finally pending the action py code until initial states of drones
                    while (not rospy.is_shutdown()):
                        if (board.controller_00.local_pos.x <= 0.1 and board.controller_00.local_pos.x >= -0.1) and \
                            (board.controller_00.local_pos.y <= 0.1 and board.controller_00.local_pos.y >= -0.1) and \
                            (board.controller_00.local_pos.z <= 3.1 and board.controller_00.local_pos.z >= 2.9):
                            pub_to_dqn.pub_reset_00.publish(data=False)
                            break
                    break
                    
            # up + positive x + negative y - Check OK
            if sub_from_dqn.action_index == 15:
                print("select the up + x-axis + negative y-axis")
                board.controller_00.action_15()
                pub_to_dqn.target_00.publish(board.controller_00.sp)
                        
                while (not rospy.is_shutdown()):
                    
                    if (abs(board.controller_00.sp.position.x - board.controller_00.local_pos.x) <= 0.2 and \
                        abs(board.controller_00.sp.position.y - board.controller_00.local_pos.y) <= 0.2 and \
                            abs(board.controller_00.sp.position.z - board.controller_00.local_pos.z) <= 0.2 and \
                                sub_from_dqn.reset == False):
                        break
                    if np.amin(sub_from_dqn.laser_array) < 3:
                        board.controller_00.stop()
                        pub_to_dqn.target_00.publish(board.controller_00.sp)
                        pub_to_dqn.pub_reset_00.publish(data=True)
                        sub_from_dqn.reset = True               
                        break
                    if -1.0 >= board.controller_00.sp.position.z or board.controller_00.sp.position.z >= 80.0:
                        board.controller_00.stop()
                        pub_to_dqn.target_00.publish(board.controller_00.sp)
                        pub_to_dqn.pub_reset_00.publish(data=True)
                        sub_from_dqn.reset = True
                        break
                    
                    continue

                if sub_from_dqn.reset == False:                
                # 4. one time, publish the reward to DQN code
                # pub_to_dqn.pub_reward.publish(pub_to_dqn.reward_msg)
                    pub_to_dqn.reward_msg.data = 1.0
                    pub_to_dqn.DQN_reward.publish(pub_to_dqn.reward_msg)
                    #pub_to_dqn.reward.publish(data=1.0)
                # 5. one time, publish the done message to DQN code
                    pub_to_dqn.done.publish(data=False)
                
                    pub_to_dqn.pending_msg = 1
                # 6. publish the pending signal message to DQN code
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)
                    #if current position is terminal, action node is publishing the true signal
                        
                    rospy.sleep(0.1)
                    pub_to_dqn.pending_msg = 0
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)                                                                   
                    break
                
                else:
                    # When simulater is restarted because of the obstacles, the agents gain negative rewards 
                    pub_to_dqn.reward_msg.data = -100.0
                    pub_to_dqn.DQN_reward.publish(pub_to_dqn.reward_msg)
                    rospy.sleep(0.1)
                    
                    # Then action py publish the done message to DQN code
                    pub_to_dqn.done.publish(data=True)
                    
                    pub_to_dqn.pending_msg = 1
                    # publish the pending signal message to DQN code
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)
                        
                    rospy.sleep(0.1)
                    pub_to_dqn.pending_msg = 0
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)

                    board.controller_00.origin()
                    pub_to_dqn.target_00.publish(board.controller_00.sp)     
                    
                    sub_from_dqn.reset = False
                    # finally pending the action py code until initial states of drones
                    while (not rospy.is_shutdown()):
                        if (board.controller_00.local_pos.x <= 0.1 and board.controller_00.local_pos.x >= -0.1) and \
                            (board.controller_00.local_pos.y <= 0.1 and board.controller_00.local_pos.y >= -0.1) and \
                            (board.controller_00.local_pos.z <= 3.1 and board.controller_00.local_pos.z >= 2.9):
                            pub_to_dqn.pub_reset_00.publish(data=False)
                            break
                    break
                    
            # up + negative x + positive y - Check OK
            if sub_from_dqn.action_index == 16:
                print("select the up + negative x-axis + y-axis")
                board.controller_00.action_16()
                pub_to_dqn.target_00.publish(board.controller_00.sp)

                while (not rospy.is_shutdown()):
                    
                    if (abs(board.controller_00.sp.position.x - board.controller_00.local_pos.x) <= 0.2 and \
                        abs(board.controller_00.sp.position.y - board.controller_00.local_pos.y) <= 0.2 and \
                            abs(board.controller_00.sp.position.z - board.controller_00.local_pos.z) <= 0.2 and \
                                sub_from_dqn.reset == False):
                        break
                    if np.amin(sub_from_dqn.laser_array) < 3:
                        board.controller_00.stop()
                        pub_to_dqn.target_00.publish(board.controller_00.sp)
                        pub_to_dqn.pub_reset_00.publish(data=True)
                        sub_from_dqn.reset = True   
                        break
                    if -1.0 >= board.controller_00.sp.position.z or board.controller_00.sp.position.z >= 80.0:
                        board.controller_00.stop()
                        pub_to_dqn.target_00.publish(board.controller_00.sp)
                        pub_to_dqn.pub_reset_00.publish(data=True)
                        sub_from_dqn.reset = True
                        break

                    continue

                if sub_from_dqn.reset == False:                               
                # 4. one time, publish the reward to DQN code
                # pub_to_dqn.pub_reward.publish(pub_to_dqn.reward_msg)
                    #pub_to_dqn.reward.publish(data=2.0)
                    pub_to_dqn.reward_msg.data = 2.0
                    pub_to_dqn.DQN_reward.publish(pub_to_dqn.reward_msg)
                    rospy.sleep(0.1)
                # 5. one time, publish the done message to DQN code
                    pub_to_dqn.done.publish(data=False)
                
                    pub_to_dqn.pending_msg = 1
                # 6. publish the pending signal message to DQN code
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)
                        
                    rospy.sleep(0.1)
                    pub_to_dqn.pending_msg = 0
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)                                                                      
                    break
                
                else:
                    # When simulater is restarted because of the obstacles, the agents gain negative rewards 
                    pub_to_dqn.reward_msg.data = -100.0
                    pub_to_dqn.DQN_reward.publish(pub_to_dqn.reward_msg)
                    rospy.sleep(0.1)
                    
                    # Then action py publish the done message to DQN code
                    pub_to_dqn.done.publish(data=True)
                    
                    pub_to_dqn.pending_msg = 1
                    # publish the pending signal message to DQN code
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)
                        
                    rospy.sleep(0.1)
                    pub_to_dqn.pending_msg = 0
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)

                    board.controller_00.origin()
                    pub_to_dqn.target_00.publish(board.controller_00.sp)      
                    
                    sub_from_dqn.reset = False
                    # finally pending the action py code until initial states of drones
                    while (not rospy.is_shutdown()):
                         if (board.controller_00.local_pos.x <= 0.1 and board.controller_00.local_pos.x >= -0.1) and \
                            (board.controller_00.local_pos.y <= 0.1 and board.controller_00.local_pos.y >= -0.1) and \
                            (board.controller_00.local_pos.z <= 3.1 and board.controller_00.local_pos.z >= 2.9):
                            pub_to_dqn.pub_reset_00.publish(data=False)
                            break
                    break
            
            # up + negative x + negative y - Check OK
            if sub_from_dqn.action_index == 17:
                print("select the up + negative x-axis + negative y-axis")
                board.controller_00.action_17()
                pub_to_dqn.target_00.publish(board.controller_00.sp)

                while (not rospy.is_shutdown()):
                    
                    if (abs(board.controller_00.sp.position.x - board.controller_00.local_pos.x) <= 0.2 and \
                        abs(board.controller_00.sp.position.y - board.controller_00.local_pos.y) <= 0.2 and \
                            abs(board.controller_00.sp.position.z - board.controller_00.local_pos.z) <= 0.2 and \
                                sub_from_dqn.reset == False):
                        break
                    if np.amin(sub_from_dqn.laser_array) < 3:
                        board.controller_00.stop()
                        pub_to_dqn.target_00.publish(board.controller_00.sp)
                        pub_to_dqn.pub_reset_00.publish(data=True)
                        sub_from_dqn.reset = True   
                        break
                    if -1.0 >= board.controller_00.sp.position.z or board.controller_00.sp.position.z >= 80.0:
                        board.controller_00.stop()
                        pub_to_dqn.target_00.publish(board.controller_00.sp)
                        pub_to_dqn.pub_reset_00.publish(data=True)
                        sub_from_dqn.reset = True
                        break

                    continue

                if sub_from_dqn.reset == False:                                        
                # 4. one time, publish the reward to DQN code
                    pub_to_dqn.reward_msg.data = 3.0
                    pub_to_dqn.DQN_reward.publish(pub_to_dqn.reward_msg)
                    rospy.sleep(0.1)
                # 5. one time, publish the done message to DQN code
                    pub_to_dqn.done.publish(data=False)
                
                    pub_to_dqn.pending_msg = 1
                # 6. publish the pending signal message to DQN code
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)
                        
                    rospy.sleep(0.1)
                    pub_to_dqn.pending_msg = 0
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)                                                                  
                    break        
                else:
                    # When simulater is restarted because of the obstacles, the agents gain negative rewards 
                    pub_to_dqn.reward_msg.data = -100.0
                    pub_to_dqn.DQN_reward.publish(pub_to_dqn.reward_msg)
                    rospy.sleep(0.1)
                    
                    # Then action py publish the done message to DQN code
                    pub_to_dqn.done.publish(data=True)
                    
                    pub_to_dqn.pending_msg = 1
                    # publish the pending signal message to DQN code
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)
                        
                    rospy.sleep(0.1)
                    pub_to_dqn.pending_msg = 0
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)

                    board.controller_00.origin()
                    pub_to_dqn.target_00.publish(board.controller_00.sp)   
                    
                    sub_from_dqn.reset = False
                    # finally pending the action py code until initial states of drones
                    while (not rospy.is_shutdown()):
                        if (board.controller_00.local_pos.x <= 0.1 and board.controller_00.local_pos.x >= -0.1) and \
                            (board.controller_00.local_pos.y <= 0.1 and board.controller_00.local_pos.y >= -0.1) and \
                            (board.controller_00.local_pos.z <= 3.1 and board.controller_00.local_pos.z >= 2.9):
                            pub_to_dqn.pub_reset_00.publish(data=False)
                            break
                    break

            # down + positive x - Check OK
            if sub_from_dqn.action_index == 18:
                print("select the down + positive x-axis")
                board.controller_00.action_18()
                pub_to_dqn.target_00.publish(board.controller_00.sp)

                while (not rospy.is_shutdown()):
                    
                    if (abs(board.controller_00.sp.position.x - board.controller_00.local_pos.x) <= 0.2 and \
                        abs(board.controller_00.sp.position.z - board.controller_00.local_pos.z) <= 0.2 and \
                            sub_from_dqn.reset == False):
                        break
                    if np.amin(sub_from_dqn.laser_array) < 3:
                        board.controller_00.stop()
                        pub_to_dqn.target_00.publish(board.controller_00.sp)
                        pub_to_dqn.pub_reset_00.publish(data=True)
                        sub_from_dqn.reset = True
                        break
                    if -1.0 >= board.controller_00.sp.position.z or board.controller_00.sp.position.z >= 80.0:
                        board.controller_00.stop()
                        pub_to_dqn.target_00.publish(board.controller_00.sp)
                        pub_to_dqn.pub_reset_00.publish(data=True)
                        sub_from_dqn.reset = True
                        break

                    continue

                if sub_from_dqn.reset == False:                
                # 4. one time, publish the reward to DQN code
                    pub_to_dqn.reward_msg.data = 0.0
                    pub_to_dqn.DQN_reward.publish(pub_to_dqn.reward_msg)
                    rospy.sleep(0.1)
                # 5. one time, publish the done message to DQN code
                    pub_to_dqn.done.publish(data=False)
                    #if current position is terminal, action node is publishing the true signal
                
                    pub_to_dqn.pending_msg = 1
                # 6. publish the pending signal message to DQN code
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)
                        
                    rospy.sleep(0.1)
                    pub_to_dqn.pending_msg = 0
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)                                                                        
                    break
                
                else:
                    # When simulater is restarted because of the obstacles, the agents gain negative rewards 
                    pub_to_dqn.reward_msg.data = -100.0
                    pub_to_dqn.DQN_reward.publish(pub_to_dqn.reward_msg)
                    rospy.sleep(0.1)
                    
                    # Then action py publish the done message to DQN code
                    pub_to_dqn.done.publish(data=True)
                    
                    pub_to_dqn.pending_msg = 1
                    # publish the pending signal message to DQN code
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)
                        
                    rospy.sleep(0.1)
                    pub_to_dqn.pending_msg = 0
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)

                    board.controller_00.origin()
                    pub_to_dqn.target_00.publish(board.controller_00.sp)  
                    
                    sub_from_dqn.reset = False
                    # finally pending the action py code until initial states of drones
                    while (not rospy.is_shutdown()):
                        if (board.controller_00.local_pos.x <= 0.1 and board.controller_00.local_pos.x >= -0.1) and \
                            (board.controller_00.local_pos.y <= 0.1 and board.controller_00.local_pos.y >= -0.1) and \
                            (board.controller_00.local_pos.z <= 3.1 and board.controller_00.local_pos.z >= 2.9):
                            pub_to_dqn.pub_reset_00.publish(data=False)
                            break
                    break
                        
            # down + positive y - Check OK
            if sub_from_dqn.action_index == 19:
                print("select the down + positive y-axis")
                board.controller_00.action_19()
                pub_to_dqn.target_00.publish(board.controller_00.sp)

                while (not rospy.is_shutdown()):
                   
                    if (abs(board.controller_00.sp.position.y - board.controller_00.local_pos.y) <= 0.2 and \
                        abs(board.controller_00.sp.position.z - board.controller_00.local_pos.z) <= 0.2 and \
                            sub_from_dqn.reset == False):
                        break
                    if np.amin(sub_from_dqn.laser_array) < 3:
                        board.controller_00.stop()
                        pub_to_dqn.target_00.publish(board.controller_00.sp)
                        pub_to_dqn.pub_reset_00.publish(data=True)
                        sub_from_dqn.reset = True
                        break
                    if -1.0 >= board.controller_00.sp.position.z or board.controller_00.sp.position.z >= 80.0:
                        board.controller_00.stop()
                        pub_to_dqn.target_00.publish(board.controller_00.sp)
                        pub_to_dqn.pub_reset_00.publish(data=True)
                        sub_from_dqn.reset = True
                        break

                    continue

                if sub_from_dqn.reset == False:                
                # 4. one time, publish the reward to DQN code
                    pub_to_dqn.reward_msg.data = 0.0
                    pub_to_dqn.DQN_reward.publish(pub_to_dqn.reward_msg)
                    rospy.sleep(0.1)
                # 5. one time, publish the done message to DQN code
                    pub_to_dqn.done.publish(data=False)
                    #if current position is terminal, action node is publishing the true signal
                
                    pub_to_dqn.pending_msg = 1
                # 6. publish the pending signal message to DQN code
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)
                        
                    rospy.sleep(0.1)
                    pub_to_dqn.pending_msg = 0
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)                                                                        
                    break
                
                else:
                    # When simulater is restarted because of the obstacles, the agents gain negative rewards 
                    pub_to_dqn.reward_msg.data = -100.0
                    pub_to_dqn.DQN_reward.publish(pub_to_dqn.reward_msg)
                    rospy.sleep(0.1)
                    
                    # Then action py publish the done message to DQN code
                    pub_to_dqn.done.publish(data=True)
                    
                    pub_to_dqn.pending_msg = 1
                    # publish the pending signal message to DQN code
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)
                        
                    rospy.sleep(0.1)
                    pub_to_dqn.pending_msg = 0
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)

                    board.controller_00.origin()
                    pub_to_dqn.target_00.publish(board.controller_00.sp)  
                    
                    sub_from_dqn.reset = False
                    # finally pending the action py code until initial states of drones
                    while (not rospy.is_shutdown()):
                        if (board.controller_00.local_pos.x <= 0.1 and board.controller_00.local_pos.x >= -0.1) and \
                            (board.controller_00.local_pos.y <= 0.1 and board.controller_00.local_pos.y >= -0.1) and \
                            (board.controller_00.local_pos.z <= 3.1 and board.controller_00.local_pos.z >= 2.9):
                            pub_to_dqn.pub_reset_00.publish(data=False)
                            break
                    break
            
            # down + negative x - Check OK
            if sub_from_dqn.action_index == 20:
                print("select the down + negative x-axis")
                board.controller_00.action_20()
                pub_to_dqn.target_00.publish(board.controller_00.sp)

                while (not rospy.is_shutdown()):
                    
                    if (abs(board.controller_00.sp.position.x - board.controller_00.local_pos.x) <= 0.2 and \
                        abs(board.controller_00.sp.position.z - board.controller_00.local_pos.z) <= 0.2 and \
                            sub_from_dqn.reset == False):
                        break
                    if np.amin(sub_from_dqn.laser_array) < 3:
                        board.controller_00.stop()
                        pub_to_dqn.target_00.publish(board.controller_00.sp)
                        pub_to_dqn.pub_reset_00.publish(data=True)
                        sub_from_dqn.reset = True
                        break
                    if -1.0 >= board.controller_00.sp.position.z or board.controller_00.sp.position.z >= 80.0:
                        board.controller_00.stop()
                        pub_to_dqn.target_00.publish(board.controller_00.sp)
                        pub_to_dqn.pub_reset_00.publish(data=True)
                        sub_from_dqn.reset = True
                        break

                    continue

                if sub_from_dqn.reset == False:                
                # 4. one time, publish the reward to DQN code
                    pub_to_dqn.reward_msg.data = 0.0
                    pub_to_dqn.DQN_reward.publish(pub_to_dqn.reward_msg)
                    rospy.sleep(0.1)
                # 5. one time, publish the done message to DQN code
                    pub_to_dqn.done.publish(data=False)
                    #if current position is terminal, action node is publishing the true signal
                
                    pub_to_dqn.pending_msg = 1
                # 6. publish the pending signal message to DQN code
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)
                        
                    rospy.sleep(0.1)
                    pub_to_dqn.pending_msg = 0
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)                                                                        
                    break
                
                else:
                    # When simulater is restarted because of the obstacles, the agents gain negative rewards 
                    pub_to_dqn.reward_msg.data = -100.0
                    pub_to_dqn.DQN_reward.publish(pub_to_dqn.reward_msg)
                    rospy.sleep(0.1)
                    
                    # Then action py publish the done message to DQN code
                    pub_to_dqn.done.publish(data=True)
                    
                    pub_to_dqn.pending_msg = 1
                    # publish the pending signal message to DQN code
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)
                        
                    rospy.sleep(0.1)
                    pub_to_dqn.pending_msg = 0
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)

                    board.controller_00.origin()
                    pub_to_dqn.target_00.publish(board.controller_00.sp)  
                    
                    sub_from_dqn.reset = False
                    # finally pending the action py code until initial states of drones
                    while (not rospy.is_shutdown()):
                        if (board.controller_00.local_pos.x <= 0.1 and board.controller_00.local_pos.x >= -0.1) and \
                            (board.controller_00.local_pos.y <= 0.1 and board.controller_00.local_pos.y >= -0.1) and \
                            (board.controller_00.local_pos.z <= 3.1 and board.controller_00.local_pos.z >= 2.9):
                            pub_to_dqn.pub_reset_00.publish(data=False)
                            break
                    break            
            
            # down + negative y - Check OK
            if sub_from_dqn.action_index == 21:
                print("select the down + negative y-axis")
                board.controller_00.action_21()
                pub_to_dqn.target_00.publish(board.controller_00.sp)

                while (not rospy.is_shutdown()):
                    
                    if (abs(board.controller_00.sp.position.y - board.controller_00.local_pos.y) <= 0.2 and \
                        abs(board.controller_00.sp.position.z - board.controller_00.local_pos.z) <= 0.2 and \
                            sub_from_dqn.reset == False):
                        break
                    if np.amin(sub_from_dqn.laser_array) < 3:
                        board.controller_00.stop()
                        pub_to_dqn.target_00.publish(board.controller_00.sp)
                        pub_to_dqn.pub_reset_00.publish(data=True)
                        sub_from_dqn.reset = True
                        break
                    if -1.0 >= board.controller_00.sp.position.z or board.controller_00.sp.position.z >= 80.0:
                        board.controller_00.stop()
                        pub_to_dqn.target_00.publish(board.controller_00.sp)
                        pub_to_dqn.pub_reset_00.publish(data=True)
                        sub_from_dqn.reset = True
                        break

                    continue

                if sub_from_dqn.reset == False:                
                # 4. one time, publish the reward to DQN code
                    pub_to_dqn.reward_msg.data = 0.0
                    pub_to_dqn.DQN_reward.publish(pub_to_dqn.reward_msg)
                    rospy.sleep(0.1)
                # 5. one time, publish the done message to DQN code
                    pub_to_dqn.done.publish(data=False)
                    #if current position is terminal, action node is publishing the true signal
                
                    pub_to_dqn.pending_msg = 1
                # 6. publish the pending signal message to DQN code
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)
                        
                    rospy.sleep(0.1)
                    pub_to_dqn.pending_msg = 0
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)                                                                        
                    break
                
                else:
                    # When simulater is restarted because of the obstacles, the agents gain negative rewards 
                    pub_to_dqn.reward_msg.data = -100.0
                    pub_to_dqn.DQN_reward.publish(pub_to_dqn.reward_msg)
                    rospy.sleep(0.1)
                    
                    # Then action py publish the done message to DQN code
                    pub_to_dqn.done.publish(data=True)
                    
                    pub_to_dqn.pending_msg = 1
                    # publish the pending signal message to DQN code
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)
                        
                    rospy.sleep(0.1)
                    pub_to_dqn.pending_msg = 0
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)

                    board.controller_00.origin()
                    pub_to_dqn.target_00.publish(board.controller_00.sp)  
                    
                    sub_from_dqn.reset = False
                    # finally pending the action py code until initial states of drones
                    while (not rospy.is_shutdown()):
                        if (board.controller_00.local_pos.x <= 0.1 and board.controller_00.local_pos.x >= -0.1) and \
                            (board.controller_00.local_pos.y <= 0.1 and board.controller_00.local_pos.y >= -0.1) and \
                            (board.controller_00.local_pos.z <= 3.1 and board.controller_00.local_pos.z >= 2.9):
                            pub_to_dqn.pub_reset_00.publish(data=False)
                            break
                    break

            # down + positive x + positive y - Check OK
            if sub_from_dqn.action_index == 22:
                print("select the down + x-axis + y-axis")
                board.controller_00.action_22()
                pub_to_dqn.target_00.publish(board.controller_00.sp)

                while (not rospy.is_shutdown()):
                    
                    if (abs(board.controller_00.sp.position.x - board.controller_00.local_pos.x) <= 0.2 and \
                        abs(board.controller_00.sp.position.y - board.controller_00.local_pos.y) <= 0.2 and \
                        abs(board.controller_00.sp.position.z - board.controller_00.local_pos.z) <= 0.2 and \
                            sub_from_dqn.reset == False):
                        break
                    if np.amin(sub_from_dqn.laser_array) < 3:
                        board.controller_00.stop()
                        pub_to_dqn.target_00.publish(board.controller_00.sp)
                        pub_to_dqn.pub_reset_00.publish(data=True)
                        sub_from_dqn.reset = True
                        break
                    if -1.0 >= board.controller_00.sp.position.z or board.controller_00.sp.position.z >= 80.0:
                        board.controller_00.stop()
                        pub_to_dqn.target_00.publish(board.controller_00.sp)
                        pub_to_dqn.pub_reset_00.publish(data=True)
                        sub_from_dqn.reset = True
                        break

                    continue

                if sub_from_dqn.reset == False:                
                # 4. one time, publish the reward to DQN code
                    pub_to_dqn.reward_msg.data = 0.0
                    pub_to_dqn.DQN_reward.publish(pub_to_dqn.reward_msg)
                    rospy.sleep(0.1)
                # 5. one time, publish the done message to DQN code
                    pub_to_dqn.done.publish(data=False)
                    #if current position is terminal, action node is publishing the true signal
                
                    pub_to_dqn.pending_msg = 1
                # 6. publish the pending signal message to DQN code
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)
                        
                    rospy.sleep(0.1)
                    pub_to_dqn.pending_msg = 0
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)                                                                        
                    break
                
                else:
                    # When simulater is restarted because of the obstacles, the agents gain negative rewards 
                    pub_to_dqn.reward_msg.data = -100.0
                    pub_to_dqn.DQN_reward.publish(pub_to_dqn.reward_msg)
                    rospy.sleep(0.1)
                    
                    # Then action py publish the done message to DQN code
                    pub_to_dqn.done.publish(data=True)
                    
                    pub_to_dqn.pending_msg = 1
                    # publish the pending signal message to DQN code
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)
                        
                    rospy.sleep(0.1)
                    pub_to_dqn.pending_msg = 0
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)

                    board.controller_00.origin()
                    pub_to_dqn.target_00.publish(board.controller_00.sp)  
                    
                    sub_from_dqn.reset = False
                    # finally pending the action py code until initial states of drones
                    while (not rospy.is_shutdown()):
                        if (board.controller_00.local_pos.x <= 0.1 and board.controller_00.local_pos.x >= -0.1) and \
                            (board.controller_00.local_pos.y <= 0.1 and board.controller_00.local_pos.y >= -0.1) and \
                            (board.controller_00.local_pos.z <= 3.1 and board.controller_00.local_pos.z >= 2.9):
                            pub_to_dqn.pub_reset_00.publish(data=False)
                            break
                    break
                    
            # down + positive x + negative y - Check OK
            if sub_from_dqn.action_index == 23:
                print("select the down + x-axis + negative y-axis")
                board.controller_00.action_23()
                pub_to_dqn.target_00.publish(board.controller_00.sp)
                        
                while (not rospy.is_shutdown()):
                    
                    if (abs(board.controller_00.sp.position.x - board.controller_00.local_pos.x) <= 0.2 and \
                        abs(board.controller_00.sp.position.y - board.controller_00.local_pos.y) <= 0.2 and \
                        abs(board.controller_00.sp.position.z - board.controller_00.local_pos.z) <= 0.2 and \
                            sub_from_dqn.reset == False):
                        break
                    if np.amin(sub_from_dqn.laser_array) < 3:
                        board.controller_00.stop()
                        pub_to_dqn.target_00.publish(board.controller_00.sp)
                        pub_to_dqn.pub_reset_00.publish(data=True)
                        sub_from_dqn.reset = True               
                        break
                    if -1.0 >= board.controller_00.sp.position.z or board.controller_00.sp.position.z >= 80.0:
                        board.controller_00.stop()
                        pub_to_dqn.target_00.publish(board.controller_00.sp)
                        pub_to_dqn.pub_reset_00.publish(data=True)
                        sub_from_dqn.reset = True
                        break

                    continue

                if sub_from_dqn.reset == False:                
                # 4. one time, publish the reward to DQN code
                # pub_to_dqn.pub_reward.publish(pub_to_dqn.reward_msg)
                    pub_to_dqn.reward_msg.data = 1.0
                    pub_to_dqn.DQN_reward.publish(pub_to_dqn.reward_msg)
                    #pub_to_dqn.reward.publish(data=1.0)
                # 5. one time, publish the done message to DQN code
                    pub_to_dqn.done.publish(data=False)
                
                    pub_to_dqn.pending_msg = 1
                # 6. publish the pending signal message to DQN code
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)
                    #if current position is terminal, action node is publishing the true signal
                        
                    rospy.sleep(0.1)
                    pub_to_dqn.pending_msg = 0
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)                                                                   
                    break
                
                else:
                    # When simulater is restarted because of the obstacles, the agents gain negative rewards 
                    pub_to_dqn.reward_msg.data = -100.0
                    pub_to_dqn.DQN_reward.publish(pub_to_dqn.reward_msg)
                    rospy.sleep(0.1)
                    
                    # Then action py publish the done message to DQN code
                    pub_to_dqn.done.publish(data=True)
                    
                    pub_to_dqn.pending_msg = 1
                    # publish the pending signal message to DQN code
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)
                        
                    rospy.sleep(0.1)
                    pub_to_dqn.pending_msg = 0
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)

                    board.controller_00.origin()
                    pub_to_dqn.target_00.publish(board.controller_00.sp)     
                    
                    sub_from_dqn.reset = False
                    # finally pending the action py code until initial states of drones
                    while (not rospy.is_shutdown()):
                        if (board.controller_00.local_pos.x <= 0.1 and board.controller_00.local_pos.x >= -0.1) and \
                            (board.controller_00.local_pos.y <= 0.1 and board.controller_00.local_pos.y >= -0.1) and \
                            (board.controller_00.local_pos.z <= 3.1 and board.controller_00.local_pos.z >= 2.9):
                            pub_to_dqn.pub_reset_00.publish(data=False)
                            break
                    break
                    
            # down + negative x + positive y - Check OK
            if sub_from_dqn.action_index == 24:
                print("select the down + negative x-axis + y-axis")
                board.controller_00.action_24()
                pub_to_dqn.target_00.publish(board.controller_00.sp)

                while (not rospy.is_shutdown()):
                    
                    if (abs(board.controller_00.sp.position.x - board.controller_00.local_pos.x) <= 0.2 and \
                        abs(board.controller_00.sp.position.y - board.controller_00.local_pos.y) <= 0.2 and \
                            abs(board.controller_00.sp.position.z - board.controller_00.local_pos.z) <= 0.2 and \
                                sub_from_dqn.reset == False):
                        break
                    if np.amin(sub_from_dqn.laser_array) < 3:
                        board.controller_00.stop()
                        pub_to_dqn.target_00.publish(board.controller_00.sp)
                        pub_to_dqn.pub_reset_00.publish(data=True)
                        sub_from_dqn.reset = True   
                        break
                    if -1.0 >= board.controller_00.sp.position.z or board.controller_00.sp.position.z >= 80.0:
                        board.controller_00.stop()
                        pub_to_dqn.target_00.publish(board.controller_00.sp)
                        pub_to_dqn.pub_reset_00.publish(data=True)
                        sub_from_dqn.reset = True
                        break

                    continue

                if sub_from_dqn.reset == False:               
                # 4. one time, publish the reward to DQN code
                # pub_to_dqn.pub_reward.publish(pub_to_dqn.reward_msg)
                    #pub_to_dqn.reward.publish(data=2.0)
                    pub_to_dqn.reward_msg.data = 2.0
                    pub_to_dqn.DQN_reward.publish(pub_to_dqn.reward_msg)
                    rospy.sleep(0.1)
                # 5. one time, publish the done message to DQN code
                    pub_to_dqn.done.publish(data=False)
                
                    pub_to_dqn.pending_msg = 1
                # 6. publish the pending signal message to DQN code
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)
                        
                    rospy.sleep(0.1)
                    pub_to_dqn.pending_msg = 0
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)                                                                      
                    break
                
                else:
                    # When simulater is restarted because of the obstacles, the agents gain negative rewards 
                    pub_to_dqn.reward_msg.data = -100.0
                    pub_to_dqn.DQN_reward.publish(pub_to_dqn.reward_msg)
                    rospy.sleep(0.1)
                    
                    # Then action py publish the done message to DQN code
                    pub_to_dqn.done.publish(data=True)
                    
                    pub_to_dqn.pending_msg = 1
                    # publish the pending signal message to DQN code
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)
                        
                    rospy.sleep(0.1)
                    pub_to_dqn.pending_msg = 0
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)

                    board.controller_00.origin()
                    pub_to_dqn.target_00.publish(board.controller_00.sp)      
                    
                    sub_from_dqn.reset = False
                    # finally pending the action py code until initial states of drones
                    while (not rospy.is_shutdown()):
                         if (board.controller_00.local_pos.x <= 0.1 and board.controller_00.local_pos.x >= -0.1) and \
                            (board.controller_00.local_pos.y <= 0.1 and board.controller_00.local_pos.y >= -0.1) and \
                            (board.controller_00.local_pos.z <= 3.1 and board.controller_00.local_pos.z >= 2.9):
                            pub_to_dqn.pub_reset_00.publish(data=False)
                            break
                    break
            
            # down + negative x + negative y - Check OK
            if sub_from_dqn.action_index == 25: 
                print("select the down + negative x-axis + negative y-axis")
                board.controller_00.action_25()
                pub_to_dqn.target_00.publish(board.controller_00.sp)
                
                while (not rospy.is_shutdown()):
                    
                    if (abs(board.controller_00.sp.position.x - board.controller_00.local_pos.x) <= 0.2 and \
                        abs(board.controller_00.sp.position.y - board.controller_00.local_pos.y) <= 0.2 and \
                            abs(board.controller_00.sp.position.z - board.controller_00.local_pos.z) <= 0.2 and \
                                sub_from_dqn.reset == False):
                        break
                    if np.amin(sub_from_dqn.laser_array) < 3:
                        board.controller_00.stop()
                        pub_to_dqn.target_00.publish(board.controller_00.sp)
                        pub_to_dqn.pub_reset_00.publish(data=True)
                        sub_from_dqn.reset = True   
                        break
                    if -1.0 >= board.controller_00.sp.position.z or board.controller_00.sp.position.z >= 80.0:
                        board.controller_00.stop()
                        pub_to_dqn.target_00.publish(board.controller_00.sp)
                        pub_to_dqn.pub_reset_00.publish(data=True)
                        sub_from_dqn.reset = True
                        break

                    continue
                        
                if sub_from_dqn.reset == False:
                
                # 4. one time, publish the reward to DQN code
                    pub_to_dqn.reward_msg.data = 3.0
                    pub_to_dqn.DQN_reward.publish(pub_to_dqn.reward_msg)
                    rospy.sleep(0.1)
                # 5. one time, publish the done message to DQN code
                    pub_to_dqn.done.publish(data=False)
                
                    pub_to_dqn.pending_msg = 1
                # 6. publish the pending signal message to DQN code
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)
                        
                    rospy.sleep(0.1)
                    pub_to_dqn.pending_msg = 0
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)                                                                  
                    break        
                else:
                    # When simulater is restarted because of the obstacles, the agents gain negative rewards 
                    pub_to_dqn.reward_msg.data = -100.0
                    pub_to_dqn.DQN_reward.publish(pub_to_dqn.reward_msg)
                    rospy.sleep(0.1)
                    
                    # Then action py publish the done message to DQN code
                    pub_to_dqn.done.publish(data=True)
                    
                    pub_to_dqn.pending_msg = 1
                    # publish the pending signal message to DQN code
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)
                        
                    rospy.sleep(0.1)
                    pub_to_dqn.pending_msg = 0
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)

                    board.controller_00.origin()
                    pub_to_dqn.target_00.publish(board.controller_00.sp)   
                    
                    sub_from_dqn.reset = False
                    # finally pending the action py code until initial states of drones
                    while (not rospy.is_shutdown()):
                        if (board.controller_00.local_pos.x <= 0.1 and board.controller_00.local_pos.x >= -0.1) and \
                            (board.controller_00.local_pos.y <= 0.1 and board.controller_00.local_pos.y >= -0.1) and \
                            (board.controller_00.local_pos.z <= 3.1 and board.controller_00.local_pos.z >= 2.9):
                            pub_to_dqn.pub_reset_00.publish(data=False)
                            break
                    break

            else: 
                continue
                    
            rospy.spin()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass