#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('cad4x_ros')
import sys
import rospy
import cv2
import numpy as np

# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *

# talker
from std_msgs.msg import String, Bool
from sensor_msgs.msg  import Image
from cv_bridge import CvBridge, CvBridgeError

# pending service
from cad4x_ros.srv import image_srv, image_srvResponse


class image_converter_00:
    
    def __init__(self):
        self.bridge = CvBridge()
        self.cv_image = None
        self.image_pub = rospy.Publisher("DQNimage_00", Image, queue_size=10)
        self.image_sub = rospy.Subscriber("/iris_0/camera_red_iris/image_raw", Image, self.callback)
        
    def callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            #print(self.cv_image)
        except CvBridgeError as e:
            print(e)
            
        #print(self.cv_image)
        
        #self.dst = cv2.resize(cv_image, dsize=(480,320), interpolation=cv2.INTER_AREA).astype(np.float32)
        #cv2.waitKey(2)
     
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)

    def image_signal_response(self, request):
        '''
        Callback function used by the service server to process requests from clients.
        '''

        #if request == 'True':
        #    image_pub.publish(self.bridge.cv2_to_imgmsg(self.cv_image, "bgr8"))
        request = 'False'
    
        return image_srvResponse(request)
      
def main():
    
    rospy.init_node('image_server', anonymous=True)
    ic = image_converter_00()
    
    image_service = rospy.Service('image_pending', image_srv, ic.image_signal_response)
    
    while (not rospy.is_shutdown()):
        try:
            rospy.spin()
        except CvBridgeError as e:
            print(e)           
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
        