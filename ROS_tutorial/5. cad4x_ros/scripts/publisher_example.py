#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Int32, Float32, Bool

def publisher():
    pub_count = rospy.Publisher('count', Int32, queue_size=10)
    pub_pending = rospy.Publisher('pending_signal', Int32, queue_size=10)
    pub_reward = rospy.Publisher('reward', Float32, queue_size=10)
    pub_done = rospy.Publisher('terminal', Bool, queue_size=10)
    rospy.init_node('publisher')
    rate = rospy.Rate(30.0)
    
    count = 1
    pend = 1
    reward = 0.0
    done = False
    count_msg = Int32()
    pending_msg = Int32()
    reward_msg = Float32()
    done_msg = Bool()
    while (not rospy.is_shutdown()):
        count_msg.data = count
        pending_msg.data = pend
        reward_msg.data = reward
        done_msg.data = done

        pub_count.publish(count_msg)
        pub_pending.publish(pending_msg)
        pub_reward.publish(reward_msg)
        pub_done.publish(done_msg)
        rospy.loginfo(count_msg)
        rospy.loginfo(pending_msg)
        rospy.loginfo(reward_msg)
        rospy.loginfo(done_msg)
        count += 1
        rate.sleep()
        
if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass