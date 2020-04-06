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

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# pending service
from cad4x_ros.srv import image_srv, image_srvResponse

class Depth_00:

    def __init__(self):
        self.bridge = CvBridge()
        self.cv_image = None
        self.image_pub = rospy.Publisher("Depth_00",Image, queue_size=10)
        self.image_sub = rospy.Subscriber("/uav0/depth/depth/image_raw", Image, self.callback)

    def callback(self,data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "passthrough")        
            #self.cv_image = self.cv_image.reshape(320,480)
            # print(self.cv_image.reshape(320, 480, 1).shape)
            #cv_image_array = self.bridge.imgmsg_to_cv2(data, "8UC1")
            #cv_image_array = np.array(cv_image, dtype = np.float32)
            #cv_image_norm = cv2.normalize(cv_image_array, cv_image_array, 0, 1, cv2.NORM_MINMAX)
        except CvBridgeError as e:
            print(e)

        #cv2.imshow("Image window", self.cv_image)
        #cv2.waitKey(3)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.cv_image, "passthrough"))
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
    
    rospy.init_node('Depth_image_node', anonymous=True)
    ic = Depth_00()
    
    image_service = rospy.Service('Depth_pending', image_srv, ic.image_signal_response)
    
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