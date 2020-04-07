#!/usr/bin/env python

import rospy
import multi_off_board as mob
import numpy as np
import math
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

class action:
   
    def __init__(self, drone_position, goal_position, step_reward, max_height):
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
        
        # Initialize the publisher message
        self.Test_resume = None
        self.action_index = None
        self.laser_array = None
        self.reset = False
        
        # Initial conditions
        self.drone_position = drone_position
        self.goal_position = goal_position
        self.origin = self.goal_position + self.drone_position
        self.distance = None
        self.ns_distance = None
        self.step_reward = step_reward
        self.max_height = max_height

        # flight mode object
        # mob = multi_off_board
        # importing the FCU controller
        self.modes_00 = mob.fcuModes_00()
        self.modes_01 = mob.fcuModes_01()
        self.modes_02 = mob.fcuModes_02()
    
        # importing the "Main" controller object
        self.cnt_00 = mob.Controller()
        self.cnt_01 = mob.Controller()
        self.cnt_02 = mob.Controller()

        # importing the off_board object 
        self.board = mob.off_board(self.modes_00, self.modes_01, self.modes_02, self.cnt_00, self.cnt_01, self.cnt_02)

        # constraints
        self.moving_sequence_x = abs(self.board.controller_00.sp.position.x - self.board.controller_00.local_pos.x)
        self.moving_sequence_y = abs(self.board.controller_00.sp.position.y - self.board.controller_00.local_pos.y)
        self.moving_sequence_z = abs(self.board.controller_00.sp.position.z - self.board.controller_00.local_pos.z)

        # set up the publisher
        self.DQN_resume = rospy.Publisher('DQN_resume', Int64, queue_size=5)
        self.DQN_reward = rospy.Publisher('DQN_reward', Float32, queue_size=5)
        self.DQN_done = rospy.Publisher('DQN_terminal', Bool, queue_size=5)
        
        self.pub_reset_00 = rospy.Publisher('Reset_00', Bool, queue_size=1)
        self.pub_target_00 = rospy.Publisher('Target_00', PositionTarget, queue_size=1)
        # self.pub_target_01 = rospy.Publisher('Target_01', PositionTarget, queue_size=1)
        # self.pub_target_02 = rospy.Publisher('Target_02', PositionTarget, queue_size=1)
                   
        # set up the subscriber
        self.DQN_action_idx = rospy.Subscriber('action_index', DQN_to_CNT, self.action_callback)

        self.sub_laser_00 = rospy.Subscriber('/uav0/laser/scan', LaserScan, self.callback_laser)        
        self.sub_reset_00 = rospy.Subscriber('Reset_00', Bool, self.callback_reset)       
        self.sub_position_00 = rospy.Subscriber('/uav0/mavros/local_position/pose', PoseStamped, self.posCb)


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
    
    def posCb(self, msg):
        self.origin= [
            self.goal_position[0] + self.drone_position[0] + msg.pose.position.x,
            self.goal_position[1] + self.drone_position[1] + msg.pose.position.y,
            self.goal_position[2] + self.drone_position[2] + msg.pose.position.z,
        ]
        self.global_position = [
            self.drone_position[0] + msg.pose.position.x,
            self.drone_position[1] + msg.pose.position.y,
            self.drone_position[2] + msg.pose.position.z,
        ]

    def move_pending(self):            
        self.board.controller_00.stop()
        self.pub_target_00.publish(self.board.controller_00.sp)
        self.pub_reset_00.publish(data=True)
        self.reset = True

    def transition_sending(self, distance, ns_distance): 

        self.distance = distance
        self.ns_distance = ns_distance

        if (self.ns_distance < self.distance):
            self.step_reward = math.sqrt(self.distance - self.ns_distance) * 0.1
        else:
            self.step_reward = -0.1

        # publish the reward to DQN code
        self.reward_msg.data = self.step_reward
        self.DQN_reward.publish(self.reward_msg)

        # publish the done message to DQN code
        # if current position is terminal, action node is publishing the true signal
        self.DQN_done.publish(data=False)
        
        # publish the pending signal message to DQN code
        self.pending_msg = 1
        self.DQN_resume.publish(self.pending_msg)
                        
        # resume and re-pending the DQN code
        self.pending_msg = 0
        self.DQN_resume.publish(self.pending_msg)   

    def terminal_sequence(self):
        # When simulater is restarted because of the obstacles, the agents gain negative rewards 
        self.reward_msg.data = -100.0
        self.DQN_reward.publish(self.reward_msg)
                    
        # Then action py publish the done message to DQN code
        self.DQN_done.publish(data=True)
                    
        # publish the pending signal message to DQN code
        self.pending_msg = 1
        self.DQN_resume.publish(self.pending_msg)

        # resume and re-pending the DQN code                
        self.pending_msg = 0
        self.DQN_resume.publish(self.pending_msg)

        # publish the origin coordinate of drone
        self.board.controller_00.origin()
        self.pub_target_00.publish(self.board.controller_00.sp)

        rospy.sleep(0.1) 
                    
        self.reset = False
        
        # finally pending the action py code until the drone reach initial states 
        while (not rospy.is_shutdown()):
            if (self.board.controller_00.local_pos.x <= 0.1 and self.board.controller_00.local_pos.x >= -0.1) and \
                (self.board.controller_00.local_pos.y <= 0.1 and self.board.controller_00.local_pos.y >= -0.1) and \
                (self.board.controller_00.local_pos.z <= 10.4 and self.board.controller_00.local_pos.z >= 9.7):
                    self.pub_reset_00.publish(data=False)
                    break 


# Main function
def main():

    # initiate node
    rospy.init_node('action_node', anonymous=True)
    
    drone = [245, -245, 0]
    goal = [0, 0, 20]
        
    step_reward = -0.1
    max_height = 45.0

    act = action(drone, goal, step_reward, max_height)
    distance = math.sqrt(pow(act.origin[0],2) + pow(act.origin[1],2) + pow(act.origin[2],2))

    while (not rospy.is_shutdown()):

        # 1. initialize the pending loop of controller.py
        act.Test_resume = 0
        print("controller node is pended")
        
        # 2. pending the action sequence
        while True:             
            rospy.sleep(0.05)
        # 3. request the pending msg from the tf_DQN.py
        # this while loop is broken, when reciving the pending message from DQN node       
            if act.Test_resume == 1:
                print("controller node is resumed")
                break
    
        while (not rospy.is_shutdown()):    
            # positive x - Check OK
            if act.action_index == 0:
                print("select the x-axis")
                act.board.controller_00.x_dir()
                act.pub_target_00.publish(act.board.controller_00.sp)

                # the pending function for waiting the drone moving in gazebo simulation
                while (not rospy.is_shutdown()):
                    if (abs(act.board.controller_00.sp.position.x - act.board.controller_00.local_pos.x) <= 0.2 and \
                        act.reset == False):
                        break
                    if np.amin(act.laser_array) < 3:
                        act.move_pending()
                        break
                    if -1.0 >= act.board.controller_00.sp.position.z or act.board.controller_00.sp.position.z >= act.max_height:
                        act.move_pending()
                        break
                    if (abs(act.global_position[0] > 250) or abs (act.global_position[1] > 250)):
                        act.move_pending()
                        break
                    continue

                if act.reset == False:
                    ns_distance = math.sqrt(pow(act.origin[0],2) + pow(act.origin[1],2) + pow(act.origin[2],2))
                # send the transition of DQN when drone moving is over
                    act.transition_sending(distance, ns_distance)
                    rospy.sleep(0.1)
                    distance = ns_distance                                                                     
                    break
                
                else:
                    act.terminal_sequence()
                    distance = math.sqrt(pow(act.origin[0],2) + pow(act.origin[1],2) + pow(act.origin[2],2))
                    break
                    
            # positive y - Check OK
            if act.action_index == 1:
                print("select the y-axis")
                act.board.controller_00.y_dir()
                act.pub_target_00.publish(act.board.controller_00.sp)
                        
                # the pending function for waiting the drone moving in gazebo simulation
                while (not rospy.is_shutdown()):
                    if (abs(act.board.controller_00.sp.position.y - act.board.controller_00.local_pos.y) <= 0.2 and \
                        act.reset == False):
                        break
                    if np.amin(act.laser_array) < 3:
                        act.move_pending()
                        break
                    if -1.0 >= act.board.controller_00.sp.position.z or act.board.controller_00.sp.position.z >= act.max_height:
                        act.move_pending()
                        break
                    if (abs(act.global_position[0] > 250) or abs (act.global_position[1] > 250)):
                        act.move_pending()
                        break
                    continue
                
                if act.reset == False:
                    ns_distance = math.sqrt(pow(act.origin[0],2) + pow(act.origin[1],2) + pow(act.origin[2],2))
                # send the transition of DQN when drone moving is over
                    act.transition_sending(distance, ns_distance)
                    rospy.sleep(0.1)
                    distance = ns_distance                                                                     
                    break
                
                else:
                    act.terminal_sequence()
                    distance = math.sqrt(pow(act.origin[0],2) + pow(act.origin[1],2) + pow(act.origin[2],2))
                    break
                    
            # negative x - Check OK
            if act.action_index == 2:
                print("select the negative x-axis")
                act.board.controller_00.neg_x_dir()
                act.pub_target_00.publish(act.board.controller_00.sp)

                # the pending function for waiting the drone moving in gazebo simulation
                while (not rospy.is_shutdown()):
                    if (abs(act.board.controller_00.sp.position.x - act.board.controller_00.local_pos.x) <= 0.2 and \
                        act.reset == False):
                        break
                    if np.amin(act.laser_array) < 3:
                        act.move_pending()
                        break
                    if -1.0 >= act.board.controller_00.sp.position.z or act.board.controller_00.sp.position.z >= act.max_height:
                        act.move_pending()
                        break
                    if (abs(act.global_position[0] > 250) or abs (act.global_position[1] > 250)):
                        act.move_pending()
                        break
                    continue
                
                if act.reset == False:
                    ns_distance = math.sqrt(pow(act.origin[0],2) + pow(act.origin[1],2) + pow(act.origin[2],2))
                # send the transition of DQN when drone moving is over
                    act.transition_sending(distance, ns_distance)
                    rospy.sleep(0.1)
                    distance = ns_distance                                                                     
                    break
                
                else:
                    act.terminal_sequence()
                    distance = math.sqrt(pow(act.origin[0],2) + pow(act.origin[1],2) + pow(act.origin[2],2))
                    break
            
            # negative y - Check OK
            if act.action_index == 3:
                print("select the negative y-axis")
                act.board.controller_00.neg_y_dir()
                act.pub_target_00.publish(act.board.controller_00.sp)
                
                # the pending function for waiting the drone moving in gazebo simulation
                while (not rospy.is_shutdown()):
                    if (abs(act.board.controller_00.sp.position.y - act.board.controller_00.local_pos.y) <= 0.2 and \
                        act.reset == False):
                        break
                    if np.amin(act.laser_array) < 3:
                        act.move_pending()
                        break
                    if -1.0 >= act.board.controller_00.sp.position.z or act.board.controller_00.sp.position.z >= act.max_height:
                        act.move_pending()
                        break
                    if (abs(act.global_position[0] > 250) or abs (act.global_position[1] > 250)):
                        act.move_pending()
                        break
                    continue
                
                if act.reset == False:
                    ns_distance = math.sqrt(pow(act.origin[0],2) + pow(act.origin[1],2) + pow(act.origin[2],2))
                # send the transition of DQN when drone moving is over
                    act.transition_sending(distance, ns_distance)
                    rospy.sleep(0.1)
                    distance = ns_distance                                                                     
                    break
                
                else:
                    act.terminal_sequence()
                    distance = math.sqrt(pow(act.origin[0],2) + pow(act.origin[1],2) + pow(act.origin[2],2))
                    break

            # positive x + positive y - Check OK
            if act.action_index == 4:
                print("select the x-axis + y-axis")
                act.board.controller_00.action_4()
                act.pub_target_00.publish(act.board.controller_00.sp)

                # the pending function for waiting the drone moving in gazebo simulation
                while (not rospy.is_shutdown()):
                    if (abs(act.board.controller_00.sp.position.x - act.board.controller_00.local_pos.x) <= 0.2 and \
                        abs(act.board.controller_00.sp.position.y - act.board.controller_00.local_pos.y) <= 0.2 and \
                            act.reset == False):
                        break
                    if np.amin(act.laser_array) < 3:
                        act.move_pending()
                        break
                    if -1.0 >= act.board.controller_00.sp.position.z or act.board.controller_00.sp.position.z >= act.max_height:
                        act.move_pending()
                        break
                    if (abs(act.global_position[0] > 250) or abs (act.global_position[1] > 250)):
                        act.move_pending()
                        break
                    continue
                
                if act.reset == False:
                    ns_distance = math.sqrt(pow(act.origin[0],2) + pow(act.origin[1],2) + pow(act.origin[2],2))
                # send the transition of DQN when drone moving is over
                    act.transition_sending(distance, ns_distance)
                    rospy.sleep(0.1)
                    distance = ns_distance                                                                     
                    break
                
                else:
                    act.terminal_sequence()
                    distance = math.sqrt(pow(act.origin[0],2) + pow(act.origin[1],2) + pow(act.origin[2],2))
                    break
                                    
            # positive x + negative y - Check OK
            if act.action_index == 5:
                print("select the x-axis and negative y-axis")
                act.board.controller_00.action_5()
                act.pub_target_00.publish(act.board.controller_00.sp)
                        
                # the pending function for waiting the drone moving in gazebo simulation
                while (not rospy.is_shutdown()):
                    if (abs(act.board.controller_00.sp.position.x - act.board.controller_00.local_pos.x) <= 0.2 and \
                        abs(act.board.controller_00.sp.position.y - act.board.controller_00.local_pos.y) <= 0.2 and \
                            act.reset == False):
                        break
                    if np.amin(act.laser_array) < 3:
                        act.move_pending()
                        break
                    if -1.0 >= act.board.controller_00.sp.position.z or act.board.controller_00.sp.position.z >= act.max_height:
                        act.move_pending()
                        break
                    if (abs(act.global_position[0] > 250) or abs (act.global_position[1] > 250)):
                        act.move_pending()
                        break
                    continue
                
                if act.reset == False:
                    ns_distance = math.sqrt(pow(act.origin[0],2) + pow(act.origin[1],2) + pow(act.origin[2],2))
                # send the transition of DQN when drone moving is over
                    act.transition_sending(distance, ns_distance)
                    rospy.sleep(0.1)
                    distance = ns_distance                                                                     
                    break
                
                else:
                    act.terminal_sequence()
                    distance = math.sqrt(pow(act.origin[0],2) + pow(act.origin[1],2) + pow(act.origin[2],2))
                    break
                    
            # negative x + positive y - Check OK
            if act.action_index == 6:
                print("select the negative x-axis and y-axis")
                act.board.controller_00.action_6()
                act.pub_target_00.publish(act.board.controller_00.sp)
                
                # the pending function for waiting the drone moving in gazebo simulation
                while (not rospy.is_shutdown()):
                    if (abs(act.board.controller_00.sp.position.x - act.board.controller_00.local_pos.x) <= 0.2 and \
                        abs(act.board.controller_00.sp.position.y - act.board.controller_00.local_pos.y) <= 0.2 and \
                            act.reset == False):
                        break
                    if np.amin(act.laser_array) < 3:
                        act.move_pending()
                        break
                    if -1.0 >= act.board.controller_00.sp.position.z or act.board.controller_00.sp.position.z >= act.max_height:
                        act.move_pending()
                        break
                    if (abs(act.global_position[0] > 250) or abs (act.global_position[1] > 250)):
                        act.move_pending()
                        break
                    continue
                
                if act.reset == False:
                    ns_distance = math.sqrt(pow(act.origin[0],2) + pow(act.origin[1],2) + pow(act.origin[2],2))
                # send the transition of DQN when drone moving is over
                    act.transition_sending(distance, ns_distance)
                    rospy.sleep(0.1)
                    distance = ns_distance                                                                     
                    break
                
                else:
                    act.terminal_sequence()
                    distance = math.sqrt(pow(act.origin[0],2) + pow(act.origin[1],2) + pow(act.origin[2],2))
                    break
            
            # negative x + negative y - Check OK
            if act.action_index == 7:
                print("select the negative x-axis and negative y-axis")
                act.board.controller_00.action_7()
                act.pub_target_00.publish(act.board.controller_00.sp)
                
                # the pending function for waiting the drone moving in gazebo simulation
                while (not rospy.is_shutdown()):
                    if (abs(act.board.controller_00.sp.position.x - act.board.controller_00.local_pos.x) <= 0.2 and \
                        abs(act.board.controller_00.sp.position.y - act.board.controller_00.local_pos.y) <= 0.2 and \
                            act.reset == False):
                        break
                    if np.amin(act.laser_array) < 3:
                        act.move_pending()
                        break
                    if -1.0 >= act.board.controller_00.sp.position.z or act.board.controller_00.sp.position.z >= act.max_height:
                        act.move_pending()
                        break
                    if (abs(act.global_position[0] > 250) or abs (act.global_position[1] > 250)):
                        act.move_pending()
                        break
                    continue
                
                if act.reset == False:
                    ns_distance = math.sqrt(pow(act.origin[0],2) + pow(act.origin[1],2) + pow(act.origin[2],2))
                # send the transition of DQN when drone moving is over
                    act.transition_sending(distance, ns_distance)
                    rospy.sleep(0.1)
                    distance = ns_distance                                                                     
                    break
                
                else:
                    act.terminal_sequence()
                    distance = math.sqrt(pow(act.origin[0],2) + pow(act.origin[1],2) + pow(act.origin[2],2))
                    break

            # up - Check OK
            if act.action_index == 8:
                print("select the up")
                act.board.controller_00.up()
                act.pub_target_00.publish(act.board.controller_00.sp)
                
                # the pending function for waiting the drone moving in gazebo simulation
                while (not rospy.is_shutdown()):
                    if (abs(act.board.controller_00.sp.position.z - act.board.controller_00.local_pos.z) <= 0.2 and \
                        act.reset == False):
                        break
                    if np.amin(act.laser_array) < 3:
                        act.move_pending()
                        break
                    if -1.0 >= act.board.controller_00.sp.position.z or act.board.controller_00.sp.position.z >= act.max_height:
                        act.move_pending()
                        break
                    if (abs(act.global_position[0] > 250) or abs (act.global_position[1] > 250)):
                        act.move_pending()
                        break
                    continue
                
                if act.reset == False:
                    ns_distance = math.sqrt(pow(act.origin[0],2) + pow(act.origin[1],2) + pow(act.origin[2],2))
                # send the transition of DQN when drone moving is over
                    act.transition_sending(distance, ns_distance)
                    rospy.sleep(0.1)
                    distance = ns_distance                                                                     
                    break
                
                else:
                    act.terminal_sequence()
                    distance = math.sqrt(pow(act.origin[0],2) + pow(act.origin[1],2) + pow(act.origin[2],2))
                    break
            
            # down - Check OK
            if act.action_index == 9:
                print("select the down")
                act.board.controller_00.down()
                act.pub_target_00.publish(act.board.controller_00.sp)
                
                # the pending function for waiting the drone moving in gazebo simulation
                while (not rospy.is_shutdown()):
                    if (abs(act.board.controller_00.sp.position.z - act.board.controller_00.local_pos.z) <= 0.2 and \
                        act.reset == False):
                        break
                    if np.amin(act.laser_array) < 3:
                        act.move_pending()
                        break
                    if -1.0 >= act.board.controller_00.sp.position.z or act.board.controller_00.sp.position.z >= act.max_height:
                        act.move_pending()
                        break
                    if (abs(act.global_position[0] > 250) or abs (act.global_position[1] > 250)):
                        act.move_pending()
                        break
                    continue
                
                if act.reset == False:
                    ns_distance = math.sqrt(pow(act.origin[0],2) + pow(act.origin[1],2) + pow(act.origin[2],2))
                # send the transition of DQN when drone moving is over
                    act.transition_sending(distance, ns_distance)
                    rospy.sleep(0.1)
                    distance = ns_distance                                                                     
                    break
                
                else:
                    act.terminal_sequence()
                    distance = math.sqrt(pow(act.origin[0],2) + pow(act.origin[1],2) + pow(act.origin[2],2))
                    break

            # up + positive x - Check OK
            if act.action_index == 10:
                print("select the up + positive x-axis")
                act.board.controller_00.action_10()
                act.pub_target_00.publish(act.board.controller_00.sp)

                # the pending function for waiting the drone moving in gazebo simulation
                while (not rospy.is_shutdown()):
                    if (abs(act.board.controller_00.sp.position.x - act.board.controller_00.local_pos.x) <= 0.2 and \
                        abs(act.board.controller_00.sp.position.z - act.board.controller_00.local_pos.z) <= 0.2 and \
                            act.reset == False):
                        break
                    if np.amin(act.laser_array) < 3:
                        act.move_pending()
                        break
                    if -1.0 >= act.board.controller_00.sp.position.z or act.board.controller_00.sp.position.z >= act.max_height:
                        act.move_pending()
                        break
                    if (abs(act.global_position[0] > 250) or abs (act.global_position[1] > 250)):
                        act.move_pending()
                        break
                    continue
                
                if act.reset == False:
                    ns_distance = math.sqrt(pow(act.origin[0],2) + pow(act.origin[1],2) + pow(act.origin[2],2))
                # send the transition of DQN when drone moving is over
                    act.transition_sending(distance, ns_distance)
                    rospy.sleep(0.1)
                    distance = ns_distance                                                                     
                    break
                
                else:
                    act.terminal_sequence()
                    distance = math.sqrt(pow(act.origin[0],2) + pow(act.origin[1],2) + pow(act.origin[2],2))
                    break
            
            # up + positive y - Check OK
            if act.action_index == 11:
                print("select the up + positive y-axis")
                act.board.controller_00.action_11()
                act.pub_target_00.publish(act.board.controller_00.sp)

                # the pending function for waiting the drone moving in gazebo simulation
                while (not rospy.is_shutdown()):
                    if (abs(act.board.controller_00.sp.position.y - act.board.controller_00.local_pos.y) <= 0.2 and \
                        abs(act.board.controller_00.sp.position.z - act.board.controller_00.local_pos.z) <= 0.2 and \
                            act.reset == False):
                        break
                    if np.amin(act.laser_array) < 3:
                        act.move_pending()
                        break
                    if -1.0 >= act.board.controller_00.sp.position.z or act.board.controller_00.sp.position.z >= act.max_height:
                        act.move_pending()
                        break
                    if (abs(act.global_position[0] > 250) or abs (act.global_position[1] > 250)):
                        act.move_pending()
                        break
                    continue
                
                if act.reset == False:
                    ns_distance = math.sqrt(pow(act.origin[0],2) + pow(act.origin[1],2) + pow(act.origin[2],2))
                # send the transition of DQN when drone moving is over
                    act.transition_sending(distance, ns_distance)
                    rospy.sleep(0.1)
                    distance = ns_distance                                                                     
                    break
                
                else:
                    act.terminal_sequence()
                    distance = math.sqrt(pow(act.origin[0],2) + pow(act.origin[1],2) + pow(act.origin[2],2))
                    break
            
            # up + negative x - Check OK
            if act.action_index == 12:
                print("select the up + negative x-axis")
                act.board.controller_00.action_12()
                act.pub_target_00.publish(act.board.controller_00.sp)

                # the pending function for waiting the drone moving in gazebo simulation
                while (not rospy.is_shutdown()):
                    if (abs(act.board.controller_00.sp.position.x - act.board.controller_00.local_pos.x) <= 0.2 and \
                        abs(act.board.controller_00.sp.position.z - act.board.controller_00.local_pos.z) <= 0.2 and \
                            act.reset == False):
                        break
                    if np.amin(act.laser_array) < 3:
                        act.move_pending()
                        break
                    if -1.0 >= act.board.controller_00.sp.position.z or act.board.controller_00.sp.position.z >= act.max_height:
                        act.move_pending()
                        break
                    if (abs(act.global_position[0] > 250) or abs (act.global_position[1] > 250)):
                        act.move_pending()
                        break
                    continue
                
                if act.reset == False:
                    ns_distance = math.sqrt(pow(act.origin[0],2) + pow(act.origin[1],2) + pow(act.origin[2],2))
                # send the transition of DQN when drone moving is over
                    act.transition_sending(distance, ns_distance)
                    rospy.sleep(0.1)
                    distance = ns_distance                                                                     
                    break
                
                else:
                    act.terminal_sequence()
                    distance = math.sqrt(pow(act.origin[0],2) + pow(act.origin[1],2) + pow(act.origin[2],2))
                    break           
            
            # up + negative y - Check OK
            if act.action_index == 13:
                print("select the up + negative y-axis")
                act.board.controller_00.action_13()
                act.pub_target_00.publish(act.board.controller_00.sp)

                # the pending function for waiting the drone moving in gazebo simulation
                while (not rospy.is_shutdown()):
                    if (abs(act.board.controller_00.sp.position.y - act.board.controller_00.local_pos.y) <= 0.2 and \
                        abs(act.board.controller_00.sp.position.z - act.board.controller_00.local_pos.z) <= 0.2 and \
                            act.reset == False):
                        break
                    if np.amin(act.laser_array) < 3:
                        act.move_pending()
                        break
                    if -1.0 >= act.board.controller_00.sp.position.z or act.board.controller_00.sp.position.z >= act.max_height:
                        act.move_pending()
                        break
                    if (abs(act.global_position[0] > 250) or abs (act.global_position[1] > 250)):
                        act.move_pending()
                        break
                    continue
                
                if act.reset == False:
                    ns_distance = math.sqrt(pow(act.origin[0],2) + pow(act.origin[1],2) + pow(act.origin[2],2))
                # send the transition of DQN when drone moving is over
                    act.transition_sending(distance, ns_distance)
                    rospy.sleep(0.1)
                    distance = ns_distance                                                                     
                    break
                
                else:
                    act.terminal_sequence()
                    distance = math.sqrt(pow(act.origin[0],2) + pow(act.origin[1],2) + pow(act.origin[2],2))
                    break
 
            # up + positive x + positive y - Check OK
            if act.action_index == 14:
                print("select the up + x-axis + y-axis")
                act.board.controller_00.action_14()
                act.pub_target_00.publish(act.board.controller_00.sp)

                # the pending function for waiting the drone moving in gazebo simulation
                while (not rospy.is_shutdown()):
                    if (abs(act.board.controller_00.sp.position.x - act.board.controller_00.local_pos.x) <= 0.2 and \
                        abs(act.board.controller_00.sp.position.y - act.board.controller_00.local_pos.y) <= 0.2 and \
                            abs(act.board.controller_00.sp.position.z - act.board.controller_00.local_pos.z) <= 0.2 and \
                                act.reset == False):
                        break
                    if np.amin(act.laser_array) < 3:
                        act.move_pending()
                        break
                    if -1.0 >= act.board.controller_00.sp.position.z or act.board.controller_00.sp.position.z >= act.max_height:
                        act.move_pending()
                        break
                    if (abs(act.global_position[0] > 250) or abs (act.global_position[1] > 250)):
                        act.move_pending()
                        break
                    continue
                
                if act.reset == False:
                    ns_distance = math.sqrt(pow(act.origin[0],2) + pow(act.origin[1],2) + pow(act.origin[2],2))
                # send the transition of DQN when drone moving is over
                    act.transition_sending(distance, ns_distance)
                    rospy.sleep(0.1)
                    distance = ns_distance                                                                     
                    break
                
                else:
                    act.terminal_sequence()
                    distance = math.sqrt(pow(act.origin[0],2) + pow(act.origin[1],2) + pow(act.origin[2],2))
                    break
                    
            # up + positive x + negative y - Check OK
            if act.action_index == 15:
                print("select the up + x-axis + negative y-axis")
                act.board.controller_00.action_15()
                act.pub_target_00.publish(act.board.controller_00.sp)
                        
                # the pending function for waiting the drone moving in gazebo simulation
                while (not rospy.is_shutdown()):
                    if (abs(act.board.controller_00.sp.position.x - act.board.controller_00.local_pos.x) <= 0.2 and \
                        abs(act.board.controller_00.sp.position.y - act.board.controller_00.local_pos.y) <= 0.2 and \
                            abs(act.board.controller_00.sp.position.z - act.board.controller_00.local_pos.z) <= 0.2 and \
                                act.reset == False):
                        break
                    if np.amin(act.laser_array) < 3:
                        act.move_pending()
                        break
                    if -1.0 >= act.board.controller_00.sp.position.z or act.board.controller_00.sp.position.z >= act.max_height:
                        act.move_pending()
                        break
                    if (abs(act.global_position[0] > 250) or abs (act.global_position[1] > 250)):
                        act.move_pending()
                        break
                    continue
                
                if act.reset == False:
                    ns_distance = math.sqrt(pow(act.origin[0],2) + pow(act.origin[1],2) + pow(act.origin[2],2))
                # send the transition of DQN when drone moving is over
                    act.transition_sending(distance, ns_distance)
                    rospy.sleep(0.1)
                    distance = ns_distance                                                                     
                    break
                
                else:
                    act.terminal_sequence()
                    distance = math.sqrt(pow(act.origin[0],2) + pow(act.origin[1],2) + pow(act.origin[2],2))
                    break
                    
            # up + negative x + positive y - Check OK
            if act.action_index == 16:
                print("select the up + negative x-axis + y-axis")
                act.board.controller_00.action_16()
                act.pub_target_00.publish(act.board.controller_00.sp)

                # the pending function for waiting the drone moving in gazebo simulation
                while (not rospy.is_shutdown()):
                    if (abs(act.board.controller_00.sp.position.x - act.board.controller_00.local_pos.x) <= 0.2 and \
                        abs(act.board.controller_00.sp.position.y - act.board.controller_00.local_pos.y) <= 0.2 and \
                            abs(act.board.controller_00.sp.position.z - act.board.controller_00.local_pos.z) <= 0.2 and \
                                act.reset == False):
                        break
                    if np.amin(act.laser_array) < 3:
                        act.move_pending()
                        break
                    if -1.0 >= act.board.controller_00.sp.position.z or act.board.controller_00.sp.position.z >= act.max_height:
                        act.move_pending()
                        break
                    if (abs(act.global_position[0] > 250) or abs (act.global_position[1] > 250)):
                        act.move_pending()
                        break
                    continue
                
                if act.reset == False:
                    ns_distance = math.sqrt(pow(act.origin[0],2) + pow(act.origin[1],2) + pow(act.origin[2],2))
                # send the transition of DQN when drone moving is over
                    act.transition_sending(distance, ns_distance)
                    rospy.sleep(0.1)
                    distance = ns_distance                                                                     
                    break
                
                else:
                    act.terminal_sequence()
                    distance = math.sqrt(pow(act.origin[0],2) + pow(act.origin[1],2) + pow(act.origin[2],2))
                    break
            
            # up + negative x + negative y - Check OK
            if act.action_index == 17:
                print("select the up + negative x-axis + negative y-axis")
                act.board.controller_00.action_17()
                act.pub_target_00.publish(act.board.controller_00.sp)

                # the pending function for waiting the drone moving in gazebo simulation
                while (not rospy.is_shutdown()):
                    if (abs(act.board.controller_00.sp.position.x - act.board.controller_00.local_pos.x) <= 0.2 and \
                        abs(act.board.controller_00.sp.position.y - act.board.controller_00.local_pos.y) <= 0.2 and \
                            abs(act.board.controller_00.sp.position.z - act.board.controller_00.local_pos.z) <= 0.2 and \
                                act.reset == False):
                        break
                    if np.amin(act.laser_array) < 3:
                        act.move_pending()
                        break
                    if -1.0 >= act.board.controller_00.sp.position.z or act.board.controller_00.sp.position.z >= act.max_height:
                        act.move_pending()
                        break
                    if (abs(act.global_position[0] > 250) or abs (act.global_position[1] > 250)):
                        act.move_pending()
                        break
                    continue
                
                if act.reset == False:
                    ns_distance = math.sqrt(pow(act.origin[0],2) + pow(act.origin[1],2) + pow(act.origin[2],2))
                # send the transition of DQN when drone moving is over
                    act.transition_sending(distance, ns_distance)
                    rospy.sleep(0.1)
                    distance = ns_distance                                                                     
                    break
                
                else:
                    act.terminal_sequence()
                    distance = math.sqrt(pow(act.origin[0],2) + pow(act.origin[1],2) + pow(act.origin[2],2))
                    break

            # down + positive x - Check OK
            if act.action_index == 18:
                print("select the down + positive x-axis")
                act.board.controller_00.action_18()
                act.pub_target_00.publish(act.board.controller_00.sp)

                # the pending function for waiting the drone moving in gazebo simulation
                while (not rospy.is_shutdown()):
                    if (abs(act.board.controller_00.sp.position.x - act.board.controller_00.local_pos.x) <= 0.2 and \
                        abs(act.board.controller_00.sp.position.z - act.board.controller_00.local_pos.z) <= 0.2 and \
                            act.reset == False):
                        break
                    if np.amin(act.laser_array) < 3:
                        act.move_pending()
                        break
                    if -1.0 >= act.board.controller_00.sp.position.z or act.board.controller_00.sp.position.z >= act.max_height:
                        act.move_pending()
                        break
                    if (abs(act.global_position[0] > 250) or abs (act.global_position[1] > 250)):
                        act.move_pending()
                        break
                    continue
                
                if act.reset == False:
                    ns_distance = math.sqrt(pow(act.origin[0],2) + pow(act.origin[1],2) + pow(act.origin[2],2))
                # send the transition of DQN when drone moving is over
                    act.transition_sending(distance, ns_distance)
                    rospy.sleep(0.1)
                    distance = ns_distance                                                                     
                    break
                
                else:
                    act.terminal_sequence()
                    distance = math.sqrt(pow(act.origin[0],2) + pow(act.origin[1],2) + pow(act.origin[2],2))
                    break
                        
            # down + positive y - Check OK
            if act.action_index == 19:
                print("select the down + positive y-axis")
                act.board.controller_00.action_19()
                act.pub_target_00.publish(act.board.controller_00.sp)

                # the pending function for waiting the drone moving in gazebo simulation
                while (not rospy.is_shutdown()):
                    if (abs(act.board.controller_00.sp.position.y - act.board.controller_00.local_pos.y) <= 0.2 and \
                        abs(act.board.controller_00.sp.position.z - act.board.controller_00.local_pos.z) <= 0.2 and \
                            act.reset == False):
                        break
                    if np.amin(act.laser_array) < 3:
                        act.move_pending()
                        break
                    if -1.0 >= act.board.controller_00.sp.position.z or act.board.controller_00.sp.position.z >= act.max_height:
                        act.move_pending()
                        break
                    if (abs(act.global_position[0] > 250) or abs (act.global_position[1] > 250)):
                        act.move_pending()
                        break
                    continue
                
                if act.reset == False:
                    ns_distance = math.sqrt(pow(act.origin[0],2) + pow(act.origin[1],2) + pow(act.origin[2],2))
                # send the transition of DQN when drone moving is over
                    act.transition_sending(distance, ns_distance)
                    rospy.sleep(0.1)
                    distance = ns_distance                                                                     
                    break
                
                else:
                    act.terminal_sequence()
                    distance = math.sqrt(pow(act.origin[0],2) + pow(act.origin[1],2) + pow(act.origin[2],2))
                    break
            
            # down + negative x - Check OK
            if act.action_index == 20:
                print("select the down + negative x-axis")
                act.board.controller_00.action_20()
                act.pub_target_00.publish(act.board.controller_00.sp)

                # the pending function for waiting the drone moving in gazebo simulation
                while (not rospy.is_shutdown()):
                    if (abs(act.board.controller_00.sp.position.x - act.board.controller_00.local_pos.x) <= 0.2 and \
                        abs(act.board.controller_00.sp.position.z - act.board.controller_00.local_pos.z) <= 0.2 and \
                            act.reset == False):
                        break
                    if np.amin(act.laser_array) < 3:
                        act.move_pending()
                        break
                    if -1.0 >= act.board.controller_00.sp.position.z or act.board.controller_00.sp.position.z >= act.max_height:
                        act.move_pending()
                        break
                    if (abs(act.global_position[0] > 250) or abs (act.global_position[1] > 250) or act.global_position[2] < 3.0):
                        act.move_pending()
                        break
                    continue
                
                if act.reset == False:
                    ns_distance = math.sqrt(pow(act.origin[0],2) + pow(act.origin[1],2) + pow(act.origin[2],2))
                # send the transition of DQN when drone moving is over
                    act.transition_sending(distance, ns_distance)
                    rospy.sleep(0.1)
                    distance = ns_distance                                                                     
                    break
                
                else:
                    act.terminal_sequence()
                    distance = math.sqrt(pow(act.origin[0],2) + pow(act.origin[1],2) + pow(act.origin[2],2))
                    break
            
            # down + negative y - Check OK
            if act.action_index == 21:
                print("select the down + negative y-axis")
                act.board.controller_00.action_21()
                act.pub_target_00.publish(act.board.controller_00.sp)

                # the pending function for waiting the drone moving in gazebo simulation
                while (not rospy.is_shutdown()):
                    if (abs(act.board.controller_00.sp.position.y - act.board.controller_00.local_pos.y) <= 0.2 and \
                        abs(act.board.controller_00.sp.position.z - act.board.controller_00.local_pos.z) <= 0.2 and \
                            act.reset == False):
                        break
                    if np.amin(act.laser_array) < 3:
                        act.move_pending()
                        break
                    if -1.0 >= act.board.controller_00.sp.position.z or act.board.controller_00.sp.position.z >= act.max_height:
                        act.move_pending()
                        break
                    if (abs(act.global_position[0] > 250) or abs (act.global_position[1] > 250)):
                        act.move_pending()
                        break
                    continue
                
                if act.reset == False:
                    ns_distance = math.sqrt(pow(act.origin[0],2) + pow(act.origin[1],2) + pow(act.origin[2],2))
                # send the transition of DQN when drone moving is over
                    act.transition_sending(distance, ns_distance)
                    rospy.sleep(0.1)
                    distance = ns_distance                                                                     
                    break
                
                else:
                    act.terminal_sequence()
                    distance = math.sqrt(pow(act.origin[0],2) + pow(act.origin[1],2) + pow(act.origin[2],2))
                    break

            # down + positive x + positive y - Check OK
            if act.action_index == 22:
                print("select the down + x-axis + y-axis")
                act.board.controller_00.action_22()
                act.pub_target_00.publish(act.board.controller_00.sp)

                # the pending function for waiting the drone moving in gazebo simulation
                while (not rospy.is_shutdown()):
                    if (abs(act.board.controller_00.sp.position.x - act.board.controller_00.local_pos.x) <= 0.2 and \
                        abs(act.board.controller_00.sp.position.y - act.board.controller_00.local_pos.y) <= 0.2 and \
                        abs(act.board.controller_00.sp.position.z - act.board.controller_00.local_pos.z) <= 0.2 and \
                            act.reset == False):
                        break
                    if np.amin(act.laser_array) < 3:
                        act.move_pending()
                        break
                    if -1.0 >= act.board.controller_00.sp.position.z or act.board.controller_00.sp.position.z >= act.max_height:
                        act.move_pending()
                        break
                    if (abs(act.global_position[0] > 250) or abs (act.global_position[1] > 250)):
                        act.move_pending()
                        break
                    continue
                
                if act.reset == False:
                    ns_distance = math.sqrt(pow(act.origin[0],2) + pow(act.origin[1],2) + pow(act.origin[2],2))
                # send the transition of DQN when drone moving is over
                    act.transition_sending(distance, ns_distance)
                    rospy.sleep(0.1)
                    distance = ns_distance                                                                     
                    break
                
                else:
                    act.terminal_sequence()
                    distance = math.sqrt(pow(act.origin[0],2) + pow(act.origin[1],2) + pow(act.origin[2],2))
                    break
                    
            # down + positive x + negative y - Check OK
            if act.action_index == 23:
                print("select the down + x-axis + negative y-axis")
                act.board.controller_00.action_23()
                act.pub_target_00.publish(act.board.controller_00.sp)
                        
                # the pending function for waiting the drone moving in gazebo simulation
                while (not rospy.is_shutdown()):
                    if (abs(act.board.controller_00.sp.position.x - act.board.controller_00.local_pos.x) <= 0.2 and \
                        abs(act.board.controller_00.sp.position.y - act.board.controller_00.local_pos.y) <= 0.2 and \
                        abs(act.board.controller_00.sp.position.z - act.board.controller_00.local_pos.z) <= 0.2 and \
                            act.reset == False):
                        break
                    if np.amin(act.laser_array) < 3:
                        act.move_pending()
                        break
                    if -1.0 >= act.board.controller_00.sp.position.z or act.board.controller_00.sp.position.z >= act.max_height:
                        act.move_pending()
                        break
                    if (abs(act.global_position[0] > 250) or abs (act.global_position[1] > 250)):
                        act.move_pending()
                        break
                    continue
                
                if act.reset == False:
                    ns_distance = math.sqrt(pow(act.origin[0],2) + pow(act.origin[1],2) + pow(act.origin[2],2))
                # send the transition of DQN when drone moving is over
                    act.transition_sending(distance, ns_distance)
                    rospy.sleep(0.1)
                    distance = ns_distance                                                                     
                    break
                
                else:
                    act.terminal_sequence()
                    distance = math.sqrt(pow(act.origin[0],2) + pow(act.origin[1],2) + pow(act.origin[2],2))
                    break
                    
            # down + negative x + positive y - Check OK
            if act.action_index == 24:
                print("select the down + negative x-axis + y-axis")
                act.board.controller_00.action_24()
                act.pub_target_00.publish(act.board.controller_00.sp)

                # the pending function for waiting the drone moving in gazebo simulation
                while (not rospy.is_shutdown()):
                    if (abs(act.board.controller_00.sp.position.x - act.board.controller_00.local_pos.x) <= 0.2 and \
                        abs(act.board.controller_00.sp.position.y - act.board.controller_00.local_pos.y) <= 0.2 and \
                        abs(act.board.controller_00.sp.position.z - act.board.controller_00.local_pos.z) <= 0.2 and \
                            act.reset == False):
                        break
                    if np.amin(act.laser_array) < 3:
                        act.move_pending()
                        break
                    if -1.0 >= act.board.controller_00.sp.position.z or act.board.controller_00.sp.position.z >= act.max_height:
                        act.move_pending()
                        break
                    if (abs(act.global_position[0] > 250) or abs (act.global_position[1] > 250)):
                        act.move_pending()
                        break
                    continue
                
                if act.reset == False:
                    ns_distance = math.sqrt(pow(act.origin[0],2) + pow(act.origin[1],2) + pow(act.origin[2],2))
                # send the transition of DQN when drone moving is over
                    act.transition_sending(distance, ns_distance)
                    rospy.sleep(0.1)
                    distance = ns_distance                                                                     
                    break
                
                else:
                    act.terminal_sequence()
                    distance = math.sqrt(pow(act.origin[0],2) + pow(act.origin[1],2) + pow(act.origin[2],2))
                    break
            
            # down + negative x + negative y - Check OK
            if act.action_index == 25: 
                print("select the down + negative x-axis + negative y-axis")
                act.board.controller_00.action_25()
                act.pub_target_00.publish(act.board.controller_00.sp)
                
                # the pending function for waiting the drone moving in gazebo simulation
                while (not rospy.is_shutdown()):
                    if (abs(act.board.controller_00.sp.position.x - act.board.controller_00.local_pos.x) <= 0.2 and \
                        abs(act.board.controller_00.sp.position.y - act.board.controller_00.local_pos.y) <= 0.2 and \
                        abs(act.board.controller_00.sp.position.z - act.board.controller_00.local_pos.z) <= 0.2 and \
                            act.reset == False):
                        break
                    if np.amin(act.laser_array) < 3:
                        act.move_pending()
                        break
                    if -1.0 >= act.board.controller_00.sp.position.z or act.board.controller_00.sp.position.z >= act.max_height:
                        act.move_pending()
                        break
                    if (abs(act.global_position[0] > 250) or abs (act.global_position[1] > 250)):
                        act.move_pending()
                        break
                    continue
                
                if act.reset == False:
                    ns_distance = math.sqrt(pow(act.origin[0],2) + pow(act.origin[1],2) + pow(act.origin[2],2))
                # send the transition of DQN when drone moving is over
                    act.transition_sending(distance, ns_distance)
                    rospy.sleep(0.1)
                    distance = ns_distance                                                                     
                    break
                
                else:
                    act.terminal_sequence()
                    distance = math.sqrt(pow(act.origin[0],2) + pow(act.origin[1],2) + pow(act.origin[2],2))
                    break

            else: 
                continue
                    
            rospy.spin()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass