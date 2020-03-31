#!/usr/bin/env python

import rospy
import numpy as np
import cv2

# import Trigger service
from cad4x_ros.srv import image_srv

from sensor_msgs.msg  import Image
from cv_bridge import CvBridge, CvBridgeError


class image_send:
    
    def __init__(self):
        self.bridge = CvBridge()
        self.dst = None
        self.image_sub = image_sub = rospy.Subscriber("DQNimage_00", Image, self.callback)
        
    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            #print(self.cv_image)
        except CvBridgeError as e:
            print(e)    
        
        self.dst = cv2.resize(cv_image, dsize=(480,320), interpolation=cv2.INTER_AREA).astype(np.float32)


def main():
    img_send = image_send()
    rospy.init_node('image_client')
        
    # wait for this service to be running
    rospy.wait_for_service('image_pending', timeout=None)

    # Create the connection to the service.
    sos_service = rospy.ServiceProxy('image_pending', image_srv)
    
    while (not rospy.is_shutdown()):
        try:
            result = sos_service('True')
            print(img_send.dst)
            print(result)
            rospy.sleep(5.0)
        except CvBridgeError as e:
            print(e)           
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
