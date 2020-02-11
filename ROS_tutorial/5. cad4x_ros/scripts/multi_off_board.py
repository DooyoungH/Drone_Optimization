#!/usr/bin/env python
# ROS python API
import rospy

# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped

# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *

# Flight modes class
# Flight mode are activated using ROS services
# Q. Is there the necessity that all fuc mode is divide to the each class function?
# A. Yes, because we can't control the serviceproxy function.  

class fcuModes_00:
    def __init__(self):
        pass

    def setTakeoff(self):
        rospy.wait_for_service('/uav0/mavros/cmd/takeoff')
    	try:
    		takeoffService = rospy.ServiceProxy('/uav0/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
    		takeoffService(altitude = 3) # <= 
    	except rospy.ServiceException, e:
    		print "Service takeoff call failed: %s"%e

    def setArm(self):
        rospy.wait_for_service('/uav0/mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('/uav0/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException, e:
            print "Service arming call failed: %s"%e

    def setDisarm(self):
        rospy.wait_for_service('/uav0/mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('/uav0/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(False)
        except rospy.ServiceException, e:
            print "Service disarming call failed: %s"%e

    def setStabilizedMode(self):
        rospy.wait_for_service('/uav0/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('/uav0/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='STABILIZED')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Stabilized Mode could not be set."%e

    def setOffboardMode(self):
        rospy.wait_for_service('/uav0/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('/uav0/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Offboard Mode could not be set."%e

    def setAltitudeMode(self):
        rospy.wait_for_service('/uav0/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('/uav0/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='ALTCTL')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Altitude Mode could not be set."%e

    def setPositionMode(self):
        rospy.wait_for_service('/uav0/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('/uav0/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='POSCTL')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Position Mode could not be set."%e

    def setAutoLandMode(self):
        rospy.wait_for_service('/uav0/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('/uav0/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.LAND')
        except rospy.ServiceException, e:
               print "service set_mode call failed: %s. Autoland Mode could not be set."%e
               
class fcuModes_01:
    def __init__(self):
        pass

    def setTakeoff(self):
        rospy.wait_for_service('/uav1/mavros/cmd/takeoff')
    	try:
    		takeoffService = rospy.ServiceProxy('/uav1/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
    		takeoffService(altitude = 3) # <= 
    	except rospy.ServiceException, e:
    		print "Service takeoff call failed: %s"%e

    def setArm(self):
        rospy.wait_for_service('/uav1/mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('/uav1/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException, e:
            print "Service arming call failed: %s"%e

    def setDisarm(self):
        rospy.wait_for_service('/uav1/mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('/uav1/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(False)
        except rospy.ServiceException, e:
            print "Service disarming call failed: %s"%e

    def setStabilizedMode(self):
        rospy.wait_for_service('/uav1/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('/uav1/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='STABILIZED')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Stabilized Mode could not be set."%e

    def setOffboardMode(self):
        rospy.wait_for_service('/uav1/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('/uav1/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Offboard Mode could not be set."%e

    def setAltitudeMode(self):
        rospy.wait_for_service('/uav1/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('/uav1/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='ALTCTL')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Altitude Mode could not be set."%e

    def setPositionMode(self):
        rospy.wait_for_service('/uav1/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('/uav1/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='POSCTL')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Position Mode could not be set."%e

    def setAutoLandMode(self):
        rospy.wait_for_service('/uav1/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('/uav1/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.LAND')
        except rospy.ServiceException, e:
               print "service set_mode call failed: %s. Autoland Mode could not be set."%e
               
class fcuModes_02:
    def __init__(self):
        pass

    def setTakeoff(self):
        rospy.wait_for_service('/uav2/mavros/cmd/takeoff')
    	try:
    		takeoffService = rospy.ServiceProxy('/uav2/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
    		takeoffService(altitude = 3) # <= 
    	except rospy.ServiceException, e:
    		print "Service takeoff call failed: %s"%e

    def setArm(self):
        rospy.wait_for_service('/uav2/mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('/uav2/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException, e:
            print "Service arming call failed: %s"%e

    def setDisarm(self):
        rospy.wait_for_service('/uav2/mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('/uav2/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(False)
        except rospy.ServiceException, e:
            print "Service disarming call failed: %s"%e

    def setStabilizedMode(self):
        rospy.wait_for_service('/uav2/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('/uav2/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='STABILIZED')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Stabilized Mode could not be set."%e

    def setOffboardMode(self):
        rospy.wait_for_service('/uav2/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('/uav2/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Offboard Mode could not be set."%e

    def setAltitudeMode(self):
        rospy.wait_for_service('/uav2/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('/uav2/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='ALTCTL')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Altitude Mode could not be set."%e

    def setPositionMode(self):
        rospy.wait_for_service('/uav2/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('/uav2/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='POSCTL')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Position Mode could not be set."%e

    def setAutoLandMode(self):
        rospy.wait_for_service('/uav2/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('/uav2/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.LAND')
        except rospy.ServiceException, e:
               print "service set_mode call failed: %s. Autoland Mode could not be set."%e

               

class Controller:
    # initialization method
    def __init__(self):
        # Drone state
        self.state = State()
        # Instantiate a setpoints message
        self.sp = PositionTarget()
        # set the flag to use position setpoints and yaw angle
        self.sp.type_mask = int('010111111000', 2)
        # LOCAL_NED
        self.sp.coordinate_frame = 1

        # We will fly at a fixed altitude for now
        # Altitude setpoint, [meters]
        self.ALT_SP = 3.0
        # update the setpoint message with the required altitude
        self.sp.position.z = self.ALT_SP
        # Step size for position update
        self.STEP_SIZE = 2.0
		# Fence. We will assume a square fence for now
        self.FENCE_LIMIT = 5.0
        
        self.clbk_position_x = 0.0
        self.clbk_position_y = 0.0

        # A Message for the current local position of the drone
        self.local_pos = Point(0.0, 0.0, 3.0)

        # initial values for setpoints
        self.sp.position.x = 0.0
        self.sp.position.y = 0.0

        # speed of the drone is set using MPC_XY_CRUISE parameter in MAVLink
        # using QGroundControl. By default it is 5 m/s.

	# Callbacks

    ## local position callback
    def posCb(self, msg):
        self.local_pos.x = msg.pose.position.x
        self.local_pos.y = msg.pose.position.y
        self.local_pos.z = msg.pose.position.z

    ## Drone State callback
    def stateCb(self, msg):
        self.state = msg

    ## Update setpoint message
    def updateSp(self):
        self.sp.position.x = self.clbk_position_x
        self.sp.position.y = self.clbk_position_y
       
    def callback(self, msg):
        self.clbk_position_x = msg.position.x
        self.clbk_position_y = msg.position.y

    def x_dir(self):
    	self.sp.position.x = self.local_pos.x + 5
    	self.sp.position.y = self.local_pos.y
     
    def neg_x_dir(self):
    	self.sp.position.x = self.local_pos.x - 5
    	self.sp.position.y = self.local_pos.y

    def y_dir(self):
    	self.sp.position.x = self.local_pos.x
    	self.sp.position.y = self.local_pos.y + 5

    def neg_y_dir(self):
    	self.sp.position.x = self.local_pos.x
    	self.sp.position.y = self.local_pos.y - 5
     
    def origin(self):
        self.sp.position.x = 0.0
        self.sp.position.y = 0.0

# Main function
def main():

    # initiate node
    rospy.init_node('off_board_arming_server', anonymous=True)

    # flight mode object
    modes_00 = fcuModes_00()
    modes_01 = fcuModes_01()
    modes_02 = fcuModes_02()

    # controller object
    cnt_00 = Controller()
    cnt_01 = Controller()
    cnt_02 = Controller()

    # ROS loop rate
    rate = rospy.Rate(20.0)

    # Subscribe to drone state
    st_sub_00 = rospy.Subscriber('/uav0/mavros/state', State, cnt_00.stateCb)
    st_sub_01 = rospy.Subscriber('/uav1/mavros/state', State, cnt_01.stateCb)
    st_sub_02 = rospy.Subscriber('/uav2/mavros/state', State, cnt_02.stateCb)

    # Subscribe to drone's local position
    sp_sub_00 = rospy.Subscriber('/uav0/mavros/local_position/pose', PoseStamped, cnt_00.posCb)
    sp_sub_01 = rospy.Subscriber('/uav1/mavros/local_position/pose', PoseStamped, cnt_01.posCb)
    sp_sub_02 = rospy.Subscriber('/uav2/mavros/local_position/pose', PoseStamped, cnt_02.posCb)
    
    #msg_sub = rospy.Subscriber('Test', PositionTarget, cnt.msgCb)
    lis_sub_00 = rospy.Subscriber('Target_00', PositionTarget, cnt_00.callback)
    lis_sub_01 = rospy.Subscriber('Target_01', PositionTarget, cnt_01.callback)
    lis_sub_02 = rospy.Subscriber('Target_02', PositionTarget, cnt_02.callback)

    # Setpoint publisher    
    sp_pub_00 = rospy.Publisher('/uav0/mavros/setpoint_raw/local', PositionTarget, queue_size=1)
    sp_pub_01 = rospy.Publisher('/uav1/mavros/setpoint_raw/local', PositionTarget, queue_size=1)
    sp_pub_02 = rospy.Publisher('/uav2/mavros/setpoint_raw/local', PositionTarget, queue_size=1)


    # Make sure the drone is armed
    while not cnt_00.state.armed:
        modes_00.setArm()
        rate.sleep()
    
    while not cnt_01.state.armed:
        modes_01.setArm()
        rate.sleep()
        
    while not cnt_02.state.armed:
        modes_02.setArm()
        rate.sleep()

    # set in takeoff mode and takeoff to default altitude (3 m)
    # modes.setTakeoff()
    # rate.sleep()

    # We need to send few setpoint messages, then activate OFFBOARD mode, to take effect
    k=0
    while k<10:
        sp_pub_00.publish(cnt_00.sp)
        sp_pub_01.publish(cnt_01.sp)
        sp_pub_02.publish(cnt_02.sp)
        rate.sleep()
        k = k + 1

    # activate OFFBOARD mode
    modes_00.setOffboardMode()
    modes_01.setOffboardMode()
    modes_02.setOffboardMode()
    
    # ROS main loop
#    i = '1'
#    while ((not rospy.is_shutdown()) and (i in ['1','2','3','4'])):
#    	print("Press the 1,2,3,4")
#        i = raw_input("Enter input: ");
#        while(1):
#            if i=='1':
#                cnt.updateSp()
#                sp_pub.publish(cnt.sp)
#                rate.sleep()
#                break
#            elif i=='2':
#                cnt.y_dir()
#                sp_pub.publish(cnt.sp)
#                rate.sleep()
#                break
#            elif i=='3':
#                cnt.x_dir()
#                sp_pub.publish(cnt.sp)
#                rate.sleep()
#                break
#            elif i=='4':
#                cnt.neg_x_dir()
#                sp_pub.publish(cnt.sp)
#                rate.sleep()
#                break
#            else: 
#                print("?")
#                break;


    # ROS main loop
    while (not rospy.is_shutdown()):
        cnt_00.updateSp()
        cnt_01.updateSp()
        cnt_02.updateSp()
        #cnt.callback()
        ### control the position using the loop
        sp_pub_00.publish(cnt_00.sp)
        sp_pub_01.publish(cnt_01.sp)
        sp_pub_02.publish(cnt_02.sp)
        #rospy.loginfo(cnt.local_pos.x)
        rate.sleep()
        ###

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass