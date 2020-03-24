#!/usr/bin/env python

import rospy
import multi_off_board as mob
# import image_search as image

# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *

# messages and services
from std_msgs.msg import String, Bool, Float32, Int64, Int8
from sensor_msgs.msg  import Image
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
        
        # Setpoint publisher    
        self.target_00 = rospy.Publisher('Target_00', PositionTarget, queue_size=1)
        self.target_01 = rospy.Publisher('Target_01', PositionTarget, queue_size=1)
        self.target_02 = rospy.Publisher('Target_02', PositionTarget, queue_size=1)
        
        
class data_subcriber:
    
    def __init__(self):
        # Initialize the subscriber messages
        self.Test_resume = None
        self.action_index = None
        self.reset = False
        
        # set up the subscriber
        self.dqn_action_idx = rospy.Subscriber('action_idx_msg', DQN_to_CNT, self.action_callback)
        self.sub_reset_00 = rospy.Subscriber('Reset_00', Bool, self.callback_reset)
        
        # Callback functions
    def action_callback(self, data):
        self.action_index = data.action
        self.Test_resume = data.pending
        
    def callback_reset(self, data):
        self.reset = data.data

# Main function
def main():

    # initiate node
    rospy.init_node('action_node', anonymous=True)
    
    rospy.wait_for_service('/gazebo/reset_world')
    reset_simulation = rospy.ServiceProxy('/gazebo/reset_world', Empty)
    
    
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
    
    # ROS main loop
    i = '1'
    j = '0'

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
                    
        print(sub_from_dqn.action_index)

        while (not rospy.is_shutdown()):    
            if sub_from_dqn.action_index == 0:
                print("select the x-axis")
                board.controller_00.x_dir()
                pub_to_dqn.target_00.publish(board.controller_00.sp)
                print(sub_from_dqn.reset)
                while (board.controller_00.sp.position.x - board.controller_00.local_pos.x >= 0.1 and sub_from_dqn.reset == False):
                    # rospy.sleep(0.05)
                    print(board.controller_00.sp.position.x - board.controller_00.local_pos.x)
                    continue
                
                if sub_from_dqn.reset == False:
                    print("x-axis moving is done")
                
                # 4. one time, publish the reward to DQN code
                    pub_to_dqn.reward_msg.data = 0.0
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
                    board.controller_00.origin()
                    pub_to_dqn.target_00.publish(board.controller_00.sp)
                    sub_from_dqn.reset = False
                    # is it necessary of pending? yes
                    # to make this parts tomorrow
                    while board.controller_00.local_pos.z <= 2.9:
                        continue
                    break
                    
            if sub_from_dqn.action_index == 1:
                print("select the y-axis")
                board.controller_00.y_dir()
                pub_to_dqn.target_00.publish(board.controller_00.sp)
                        
                while (board.controller_00.sp.position.y - board.controller_00.local_pos.y >= 0.1 and sub_from_dqn.reset == False):
                    print(board.controller_00.sp.position.y - board.controller_00.local_pos.y)
                    continue
                
                if sub_from_dqn.reset == False:
                    print("y-axis moving is done")
                
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
                        
                    rospy.sleep(0.1)
                    pub_to_dqn.pending_msg = 0
                    pub_to_dqn.DQN_resume.publish(pub_to_dqn.pending_msg)                                                                   
                    break
                
                else:
                    board.controller_00.origin()
                    pub_to_dqn.target_00.publish(board.controller_00.sp)
                    sub_from_dqn.reset = False
                    while board.controller_00.local_pos.z <= 2.9:
                        continue
                    break
                    
            if sub_from_dqn.action_index == 2:
                print("select the negative x-axis")
                board.controller_00.neg_x_dir()
                pub_to_dqn.target_00.publish(board.controller_00.sp)
                while (board.controller_00.sp.position.x - board.controller_00.local_pos.x <= -0.1 and sub_from_dqn.reset == False):
                    print(board.controller_00.sp.position.x - board.controller_00.local_pos.x)
                    continue
                
                if sub_from_dqn.reset == False:
                    print("negative x-axis moving is done")
                
                # 4. one time, publish the reward to DQN code
                # pub_to_dqn.pub_reward.publish(pub_to_dqn.reward_msg)
                    #pub_to_dqn.reward.publish(data=2.0)
                    pub_to_dqn.reward_msg.data = 2.0
                    pub_to_dqn.DQN_reward.publish(pub_to_dqn.reward_msg)
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
                    board.controller_00.origin()
                    pub_to_dqn.target_00.publish(board.controller_00.sp)
                    sub_from_dqn.reset = False
                    while board.controller_00.local_pos.z <= 2.9:
                        continue
                    break
                        
            if sub_from_dqn.action_index == 3:
                print("select the negative y-axis")
                board.controller_00.neg_y_dir()
                pub_to_dqn.target_00.publish(board.controller_00.sp)
                while (board.controller_00.sp.position.y - board.controller_00.local_pos.y >= -0.1 and sub_from_dqn.reset == False):
                    print(board.controller_00.sp.position.x - board.controller_00.local_pos.x)
                    continue
                        
                if sub_from_dqn.reset == False:
                    print("negative y-axis moving is done")
                
                # 4. one time, publish the reward to DQN code
                # pub_to_dqn.pub_reward.publish(pub_to_dqn.reward_msg)
                    #pub_to_dqn.reward.publish(data=3.0)
                    pub_to_dqn.reward_msg.data = 3.0
                    pub_to_dqn.DQN_reward.publish(pub_to_dqn.reward_msg)
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
                    board.controller_00.origin()
                    pub_to_dqn.target_00.publish(board.controller_00.sp)
                    sub_from_dqn.reset = False
                    while board.controller_00.local_pos.z <= 2.9:
                        continue
                    break
                    
            else: 
                continue
                    
            rospy.spin()
            # break

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass