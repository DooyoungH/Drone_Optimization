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
        self.reset = False
        
        self.reset_simulation = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        self.pub_target_00 = rospy.Publisher('Target_00', PositionTarget, queue_size=1)
        self.pub_reset_00 = rospy.Publisher('Reset_00', Bool, queue_size=1)
        self.sub_reset_00 = rospy.Subscriber('Reset_00', Bool, self.callback_reset)

        # flight mode object
        self.modes_00 = mob.fcuModes_00()
        self.modes_01 = mob.fcuModes_01()
        self.modes_02 = mob.fcuModes_02()
        
        # controller object
        self.cnt_00 = mob.Controller()
        self.cnt_01 = mob.Controller()
        self.cnt_02 = mob.Controller()    
    
        self.board = mob.off_board(self.modes_00, self.modes_01, self.modes_02, self.cnt_00, self.cnt_01, self.cnt_02)
    
    def callback_reset(self, data):
        self.reset = data.data

def main():

    rospy.init_node('reset_node', anonymous=True)
    rospy.wait_for_service('/gazebo/reset_world')

    rst = reset()
    
    while (not rospy.is_shutdown()):
        while (not rospy.is_shutdown() and rst.reset != True):
            if (rst.reset == True):
                print("reset start!")
                rst.board.pub_reset_00.publish(True)
                rst.board.controller_00.sp.position.z = 0
                rst.reset_simulation()
                break
    
        while rst.board.controller_00.state.system_status == 4:
    	    rst.pub_target_00.publish(rst.board.controller_00.sp)
    	    rst.board.modes_00.setLandMod()
            rst.board.modes_00.setDisarm()
            rst.board.rate.sleep()
    
        rst.reset_simulation()
    
        # Make sure the drone is armed
        while not rst.board.controller_00.state.armed:
            rst.board.modes_00.setArm()
            rst.board.rate.sleep()
    
        # We need to send few setpoint messages, then activate OFFBOARD mode, to take effect
        k=0
        rst.board.controller_00.sp.position.x = 0
        rst.board.controller_00.sp.position.y = 0
        rst.board.controller_00.sp.position.z = 3
    
        rst.board.controller_00.sp.velocity.x = 0.000001
        rst.board.controller_00.sp.velocity.y = 0.000001
    
        while k<10:
            rst.board.pub_position_00.publish(rst.board.controller_00.sp)
    	    rst.pub_target_00.publish(rst.board.controller_00.sp)
        
            rst.board.rate.sleep()
            k = k + 1

        # activate OFFBOARD mode
        rst.board.modes_00.setOffboardMode()

        rospy.sleep(0.1)

        rst.reset = False
        rst.board.pub_reset_00.publish(data = False)
        print('Done')

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
		pass