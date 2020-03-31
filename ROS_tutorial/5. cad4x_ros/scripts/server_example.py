#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('cad4x_ros')
import sys

# import Trigger service
from cad4x_ros.srv import image_srv, image_srvResponse

import rospy

import cv2
import numpy as np

# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *

# import Trigger service
from cad4x_ros.srv import image_srv

# talker
from std_msgs.msg import String
from sensor_msgs.msg  import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter_00:
    
    def __init__(self):
        self.bridge = CvBridge()
        self.dst = np.empty([380,420,3])
        print(self.dst.shape)
        self.img_data = Image()
        self.image_sub = rospy.Subscriber("/iris_0/camera_red_iris/image_raw", Image, self.callback)
        
    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.img_data = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        self.dst = cv2.resize(cv_image, dsize=(480,320), interpolation=cv2.INTER_AREA).astype(np.float32)
        # cv2.waitKey(1)

def trigger_response(request):
    '''
    Callback function used by the service server to process requests from clients.
    '''
    ic = image_converter_00()
    
    print(request)
    image = ic.img_data
    print(image.shape)
        
    if request == 'True':
        return image_srvResponse(image)
            
def main():
    rospy.init_node('image_capture')

    image_service = rospy.Service('/fake_911', image_srv, trigger_response)
    
    rospy.spin()
           
        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass