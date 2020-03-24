#!/usr/bin/env python

import rospy
import image_search as image

from std_msgs.msg import Int32, Float32, Bool
from sensor_msgs.msg  import Image

class subscribe:
    
    def __init__(self):
        #self.processing = False
        #self.new_msg = False
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
    
def listener():
    
    #rospy.init_node('listener', anonymous=True)
    ic = image.image_converter_00()
    sb = subscribe()
    rospy.init_node('listener', anonymous=True)
    
    # need the delay?
    #rospy.sleep(1/20)

    rate = rospy.Rate(20.0)

    while (not rospy.is_shutdown()):
        # simulate a process that take 0.2 seconds
        # rospy.loginfo(msg)
        rospy.loginfo(sb.count)
        rospy.loginfo(sb.pending)
        rospy.loginfo(sb.reward)
        rospy.loginfo(sb.done)
        rate.sleep()
                  
    
if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass