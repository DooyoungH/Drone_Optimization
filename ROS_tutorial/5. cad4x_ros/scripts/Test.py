#!/usr/bin/env python

import rospy
import multi_off_board
# import image_search as image

# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *

# talker
from std_msgs.msg import String, Bool, Float32, Int64
from sensor_msgs.msg  import Image

class data_publisher:
    
    def __init__(self):
        # Initialize the message values
        self.resume = None
        self.count = 1
        self.count_msg = Int64()
        self.count_msg.data = self.count
        self.pend = 1
        self.pending_msg = Int64()
        self.pending_msg.data = self.pend
        self.n_state = None
        self.n_state_msg = Image()
        self.n_state_msg.data = self.n_state  
        self.reward = 5.0
        self.reward_msg = Float32()
        self.reward_msg.data = self.reward
        self.done = False
        self.done_msg = Bool()
        self.done_msg.data = self.done
        self.action_index = 0

        # set up the publisher
        self.pub_count = rospy.Publisher('count', Int64, queue_size=5)
        self.pub_DQN_resume = rospy.Publisher('DQN_resume', Int64, queue_size=5)
        self.pub_reward = rospy.Publisher('reward', Float32, queue_size=5)
        self.pub_done = rospy.Publisher('terminal', Bool, queue_size=5)
        
        # set up the subscriber
        self.act_sub_00 = rospy.Subscriber('action_idx_msg', Int64, self.action_callback)
        self.test_resume_sub_00 = rospy.Subscriber('test_resume_msg', Int64, self.pending_callback)


    # Callback functions
    
    def action_callback(self, data):
        self.action_index = data.data
        
    def pending_callback(self, data):
        self.resume = data.data



# Main function
def main():

    # initiate node
    rospy.init_node('control_node', anonymous=True)
    
    # controller object
    cnt_00 = multi_off_board.Controller()
    cnt_01 = multi_off_board.Controller()
    cnt_02 = multi_off_board.Controller()
    
    # Instantiate the setpoint's messages
    cnt_00.sp = PositionTarget()
    cnt_01.sp = PositionTarget()
    cnt_02.sp = PositionTarget()
    
    pub = data_publisher()
    
    # ROS loop rate
    # rate = rospy.Rate(20.0)
    
    # Subscribe to drone state
    st_sub_00 = rospy.Subscriber('/uav0/mavros/state', State, cnt_00.stateCb)
    st_sub_01 = rospy.Subscriber('/uav1/mavros/state', State, cnt_01.stateCb)
    st_sub_02 = rospy.Subscriber('/uav2/mavros/state', State, cnt_02.stateCb)

    # Subscribe to drone's local position
    sp_sub_00 = rospy.Subscriber('/uav0/mavros/local_position/pose', PoseStamped, cnt_00.posCb)
    sp_sub_01 = rospy.Subscriber('/uav1/mavros/local_position/pose', PoseStamped, cnt_01.posCb)
    sp_sub_02 = rospy.Subscriber('/uav2/mavros/local_position/pose', PoseStamped, cnt_02.posCb)

    # Setpoint publisher    
    target_pub_00 = rospy.Publisher('Target_00', PositionTarget, queue_size=1)
    target_pub_01 = rospy.Publisher('Target_01', PositionTarget, queue_size=1)
    target_pub_02 = rospy.Publisher('Target_02', PositionTarget, queue_size=1)
    
    # ROS main loop
    i = '1'
    j = '0'
    k = '0'
    
    while ((not rospy.is_shutdown()) and (k in ['0','1','2','3'])):
        print("Please the selct mode (1: DQN, 2: manual, 3:cancel)")
        k = raw_input("Enter input: ");
        
        while(1):
            if k == '1':
                print("DQN mode")
                rospy.sleep(0.5)
                print(pub.action_index)
                # is the tf_DQN pending? Yes, control the drone
                # NO, wating the pending signal
                while (not rospy.is_shutdown()):    
                    if pub.action_index == 0:
                        print("select the x-axis")
                        cnt_00.x_dir()
                        target_pub_00.publish(cnt_00.sp)
                        # rospy.loginfo(cnt_00.sp)
                        while (cnt_00.sp.position.x - cnt_00.local_pos.x >= 0.15):
                            print(cnt_00.sp.position.x - cnt_00.local_pos.x)
                            continue
                        
                        # 1. one time reward publish
                        # 2. one time done message publish
                        # 3. pending sinal publish
                        pub.pub_reward.publish(pub.reward_msg)
                        pub.pub_done.publish(pub.done_msg)
                        pub.pending_msg = 1
                        pub.pub_DQN_resume.publish(pub.pending_msg)
                        
                        rospy.sleep(0.5)
                        pub.pending_msg = 0
                        pub.pub_DQN_resume.publish(pub.pending_msg)
                        
                        pending = 0
                        print("Pending")
                        
                        while True:
                            pending = pub.resume # transfer the integer '1'       
                
                            if pending == 1:
                                break
                        
                        print("resume")                                            
                        break
                    
                    if pub.act_sub_00 == 1:
                        print("select the y-axis")
                        cnt_00.y_dir()
                        target_pub_00.publish(cnt_00.sp)
                        rospy.loginfo(cnt_00.sp)
                        while (tp.position.x - sp_sub_00.local_pos.x >= 0.2):
                            continue
                        pub.pub_reward.publish(pub.reward_msg.data)
                        pub.pub_done.publish(pub.done_msg.data)
                        pub.pub_pending.publish(pub.pending_msg.data)
                        rate.sleep()
                        break
                    if pub.act_sub_00 == 2:
                        print("select the negative x-axis")
                        cnt_00.neg_x_dir()
                        target_pub_00.publish(cnt_00.sp)
                        rospy.loginfo(cnt_00.sp)
                        while (tp.position.x - sp_sub_00.local_pos.x >= -0.2):
                            continue
                        pub.pub_reward.publish(pub.reward_msg.data)
                        pub.pub_done.publish(pub.done_msg.data)
                        pub.pub_pending.publish(pub.pending_msg.data)
                        rate.sleep()
                        break
                    if pub.act_sub_00 == 3:
                        print("select the negative y-axis")
                        cnt_00.neg_y_dir()
                        target_pub_00.publish(cnt_00.sp)
                        rospy.loginfo(cnt_00.sp)
                        while (tp.position.y - sp_sub_00.local_pos.y >= -0.2):
                            continue
                        pub.pub_reward.publish(pub.reward_msg.data)
                        pub.pub_done.publish(pub.done_msg.data)
                        pub.pub_pending.publish(pub.pending_msg.data)
                        rate.sleep()
                        break
                    ## home point
                    else: 
                        print("error")
                        break
                    rospy.spin()
                    break
            '''        
            if k == '2':
                while ((not rospy.is_shutdown()) and (i in ['1','2','3'])):
                    print("Please the select the drone (1,2,3)")
                    i = raw_input("Enter input: ");
                    while(1):
                        if i == '1':
                            while ((not rospy.is_shutdown()) and (j in ['1','2','3','4','0'])):
                                print("select the motion (1,2,3,4, and 0)")
                                j = raw_input("Enter input: ");
                                while(1):
                                    if j == '1':
                                        cnt_00.x_dir()
                                        target_pub_00.publish(cnt_00.sp)
                                        rospy.loginfo(cnt_00.sp)
                                        #while (cnt_00.sp.position.x - cnt_00.local_pos.x >= 0.2):
                                        #    print(cnt_00.sp.position.x - cnt_00.local_pos.x)
                                        #    continue
                                        pub.pub_reward.publish(pub.reward_msg.data)
                                        pub.pub_done.publish(pub.done_msg.data)
                                        pub.pub_pending.publish(pub.pending_msg.data)
                                        rate.sleep()
                                        break
                                    if j == '2':
                                        cnt_00.y_dir()
                                        target_pub_00.publish(cnt_00.sp)
                                        rospy.loginfo(cnt_00.sp)
                                        rate.sleep()
                                        break
                                    if j == '3':
                                        cnt_00.neg_x_dir()
                                        target_pub_00.publish(cnt_00.sp)
                                        rospy.loginfo(cnt_00.sp)
                                        rate.sleep()
                                        break
                                    if j == '4':
                                        cnt_00.neg_y_dir()
                                        target_pub_00.publish(cnt_00.sp)
                                        rospy.loginfo(cnt_00.sp)
                                        rate.sleep()
                                        break
                                    if j == '0':
                                        cnt_00.origin()
                                        target_pub_00.publish(cnt_00.sp)
                                        rospy.loginfo(cnt_00.sp)
                                        rate.sleep()
                                        break
                                break
                        if i == '2':
                            while ((not rospy.is_shutdown()) and (j in ['1','2','3','4','0'])):
                                print("select the motion (1,2,3,4, and 0)")
                                j = raw_input("Enter input: ");
                                while(1):
                                    if j == '1':
                                        cnt_01.x_dir()
                                        target_pub_01.publish(cnt_01.sp)
                                        rospy.loginfo(cnt_01.sp)
                                        rate.sleep()
                                        break
                                    if j == '2':
                                        cnt_01.y_dir()
                                        target_pub_01.publish(cnt_01.sp)
                                        rospy.loginfo(cnt_01.sp)
                                        rate.sleep()
                                        break
                                    if j == '3':
                                        cnt_01.neg_x_dir()
                                        target_pub_01.publish(cnt_01.sp)
                                        rospy.loginfo(cnt_01.sp)
                                        rate.sleep()
                                        break
                                    if j == '4':
                                        cnt_01.neg_y_dir()
                                        target_pub_01.publish(cnt_01.sp)
                                        rospy.loginfo(cnt_01.sp)
                                        rate.sleep()
                                        break
                                    if j == '0':
                                        cnt_01.origin()
                                        target_pub_01.publish(cnt_01.sp)
                                        rospy.loginfo(cnt_01.sp)
                                        rate.sleep()
                                        break
                                break
                        if i == '3':
                            while ((not rospy.is_shutdown()) and (j in ['1','2','3','4','0'])):
                                print("select the motion (1,2,3,4, and 0)")
                                j = raw_input("Enter input: ");
                                while(1):
                                    if j == '1':
                                        cnt_02.x_dir()
                                        target_pub_02.publish(cnt_02.sp)
                                        rospy.loginfo(cnt_02.sp)
                                        rate.sleep()
                                        break
                                    if j == '2':
                                        cnt_02.y_dir()
                                        target_pub_02.publish(cnt_02.sp)
                                        rospy.loginfo(cnt_02.sp)
                                        rate.sleep()
                                        break
                                    if j == '3':
                                        cnt_02.neg_x_dir()
                                        target_pub_02.publish(cnt_02.sp)
                                        rospy.loginfo(cnt_02.sp)
                                        rate.sleep()
                                        break
                                    if j == '4':
                                        cnt_02.neg_y_dir()
                                        target_pub_02.publish(cnt_02.sp)
                                        rospy.loginfo(cnt_02.sp)
                                        rate.sleep()
                                        break
                                    if j == '0':
                                        cnt_02.origin()
                                        target_pub_02.publish(cnt_02.sp)
                                        rospy.loginfo(cnt_02.sp)
                                        rate.sleep()
                                        break
                                break
                        else: 
                            print("error")
                            break
                '''

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass