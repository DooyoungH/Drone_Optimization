#!/usr/bin/env python

import rospy
import multi_off_board as mob

# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *

from std_srvs.srv import Empty
from std_msgs.msg import Bool

class reset:
    def __init__(self):
        # set up the values
        self.reset = False
        
        # set up the publisher
        self.pub_target_00 = rospy.Publisher('Target_00', PositionTarget, queue_size=1)
        self.pub_reset_00 = rospy.Publisher('Reset_00', Bool, queue_size=1)
        
        # set up the subscriber
        self.sub_reset_00 = rospy.Subscriber('Reset_00', Bool, self.callback_reset)
    
    def callback_reset(self, data):
        self.reset = data.data

def main():
    # initiate the reset node
    rospy.init_node('reset_node', anonymous=True)

    # flight mode object
    modes_00 = mob.fcuModes_00()
    modes_01 = mob.fcuModes_01()
    modes_02 = mob.fcuModes_02()
        
    # controller object
    cnt_00 = mob.Controller()
    cnt_01 = mob.Controller()
    cnt_02 = mob.Controller() 

    board = mob.off_board(modes_00, modes_01, modes_02, cnt_00, cnt_01, cnt_02) 
    
    rst = reset()
    print('reset node is initiated')

    while (not rospy.is_shutdown() and rst.reset != True):
        if (rst.reset == True):
            print("reset start!")
            rst.pub_reset_00.publish(True)
            break

    while (not rospy.is_shutdown()):
        if (board.controller_00.local_pos.x <= 0.1) and \
            (board.controller_00.local_pos.y <= 0.1) and \
            (board.controller_00.local_pos.z <= 3.1):
            break

    rst.reset = False
    rst.pub_reset_00.publish(data = False)
    print('Done')
       
    rospy.sleep(5.0)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass