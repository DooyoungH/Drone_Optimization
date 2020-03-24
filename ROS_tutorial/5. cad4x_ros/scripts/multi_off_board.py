#!/usr/bin/env python
# ROS python API
import rospy

# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped
from std_srvs.srv import Empty
from std_msgs.msg import Bool

# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *

# Flight modes class
# Flight mode are activated using ROS services
# Q. Is there the necessity that all fuc mode is divided to the each class function?
# A. Yes, because we can't control the numbering of serviceproxy function.  

class fcuModes_00:
    def __init__(self):
        pass

    def setTakeoff(self):
        rospy.wait_for_service('/uav0/mavros/cmd/takeoff')
    	try:
    		takeoffService = rospy.ServiceProxy('/uav0/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
    		takeoffService(altitude = 3) 
    	except rospy.ServiceException, e:
    		print "Service takeoff call failed: %s"%e
      
    def setLandMod(self):
        rospy.wait_for_service('/uav0/mavros/cmd/land')
        try:
            landService = rospy.ServiceProxy('/uav0/mavros/cmd/land', mavros_msgs.srv.CommandTOL)
            isLanding = landService(altitude = 0, latitude = 0, longitude =0, min_pitch = 0, yaw = 0)
        except rospy.ServiceException, e:
            print "service land call failed: %s. The vehicle cannot land "%e

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
        self.ALT_SP = 3
        # update the setpoint message with the required altitude
        self.sp.position.z = self.ALT_SP
        # Step size for position update
        self.STEP_SIZE = 2.0
		# Fence. We will assume a square fence for now
        self.FENCE_LIMIT = 20.0
        
        self.clbk_position_x = 0.0
        self.clbk_position_y = 0.0
        self.clbk_position_z = 3
        
        self.clbk_velocity_x = 0.0
        self.clbk_velocity_y = 0.0 

        # A Message for the current local position of the drone
        self.local_pos = Point(0.0, 0.0, 0)

        # initial values for setpoints
        self.sp.position.x = 0.0
        self.sp.position.y = 0.0
        
        self.sp.velocity.x = 0.0
        self.sp.velocity.y = 0.0

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
        self.sp.position.z = self.clbk_position_z
        
        self.sp.velocity.x = self.clbk_velocity_x                               # control the velocity of drones
        self.sp.velocity.y = self.clbk_velocity_y
       
    def callback(self, msg):
        self.clbk_position_x = msg.position.x
        self.clbk_position_y = msg.position.y
        self.clbk_position_z = msg.position.z
        
        self.clbk_velocity_x = msg.velocity.x
        self.clbk_velocity_y = msg.velocity.y
        self.clbk_velocity_z = msg.velocity.z

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
        self.local_pos.x = 0.0
        self.local_pos.y = 0.0
        self.sp.position.x = 0.0
        self.sp.position.y = 0.0
        
   
class off_board:
    
    def __init__(self,fcuModes_00, fcuModes_01, fcuModes_02, Controller_00, Controller_01, Controller_02):

        # flight mode object
        self.modes_00 = fcuModes_00
        self.modes_01 = fcuModes_01
        self.modes_02 = fcuModes_02
        
        # controller object
        self.controller_00 = Controller_00
        self.controller_01 = Controller_01
        self.controller_02 = Controller_02

        # Subscribe to drone state
        self.sub_state_00 = rospy.Subscriber('/uav0/mavros/state', State, self.controller_00.stateCb)
        self.sub_state_01 = rospy.Subscriber('/uav1/mavros/state', State, self.controller_01.stateCb)
        self.sub_state_02 = rospy.Subscriber('/uav2/mavros/state', State, self.controller_02.stateCb)

        # Subscribe to drone's local position
        self.sub_position_00 = rospy.Subscriber('/uav0/mavros/local_position/pose', PoseStamped, self.controller_00.posCb)
        self.sub_position_01 = rospy.Subscriber('/uav1/mavros/local_position/pose', PoseStamped, self.controller_01.posCb)
        self.sub_position_02 = rospy.Subscriber('/uav2/mavros/local_position/pose', PoseStamped, self.controller_02.posCb)
        
        # Subscribe to drone's velocity
    
        #msg_sub = rospy.Subscriber('Test', PositionTarget, cnt.msgCb)
        self.sub_target_00 = rospy.Subscriber('Target_00', PositionTarget, self.controller_00.callback)
        self.sub_target_01 = rospy.Subscriber('Target_01', PositionTarget, self.controller_01.callback)
        self.sub_target_02 = rospy.Subscriber('Target_02', PositionTarget, self.controller_02.callback)

        # Setpoint publisher    
        self.pub_position_00 = rospy.Publisher('/uav0/mavros/setpoint_raw/local', PositionTarget, queue_size=1)
        self.pub_position_01 = rospy.Publisher('/uav1/mavros/setpoint_raw/local', PositionTarget, queue_size=1)
        self.pub_position_02 = rospy.Publisher('/uav2/mavros/setpoint_raw/local', PositionTarget, queue_size=1)

        # reset signal from reset code
        self.pub_reset_00 = rospy.Publisher('Reset_00', Bool, queue_size=1)
        self.pub_reset_01 = rospy.Publisher('Reset_01', Bool, queue_size=1)
        self.pub_reset_02 = rospy.Publisher('Reset_02', Bool, queue_size=1)
        self.sub_reset_00 = rospy.Subscriber('Reset_00', Bool, self.reset_callback)
        self.sub_reset_01 = rospy.Subscriber('Reset_01', Bool, self.reset_callback)
        self.sub_reset_02 = rospy.Subscriber('Reset_02', Bool, self.reset_callback)

        self.reset = None

        # ROS loop rate
        self.rate = rospy.Rate(20.0)
        
    def reset_callback(self, msg):
        self.reset = msg.data
            
    def env_reset(self):
        rospy.wait_for_service('/gazebo/reset_world')
        reset_simulation = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        
        self.pub_reset_00.publish(data=True)
        
        self.controller_00.sp.position.z = 0
        reset_simulation()                

        # k=0
        # self.controller_00.sp.position.z = 3
        # while k<10:
        #    self.pub_position_00.publish(self.controller_00.sp)
        #    rospy.sleep(0.1)
        #    k = k + 1
 
        #k=0
        # while k<10:
        #    self.controller_00.sp.position.z = 0.0
        #    reset_simulation()
        #    self.rate.sleep()
        #    k = k + 1
        
        #reset_simulation()
        
        while True:
            # reset_simulation()
            self.controller_00.updateSp()
            self.rate.sleep()
            
            print(self.controller_00.state.system_status)
            if self.controller_00.state.system_status == 3:
                break
        
        # Make sure the drone is Disarmed
        # while self.controller_00.state.armed:
        #    self.modes_00.setDisarm()
        #    rospy.sleep(0.2)

        # Then, make sure the drone is re-armed
        # while not self.controller_00.state.armed:
        #    self.modes_00.setArm()
        #    rospy.sleep(0.2)
                
        # We need to send few setpoint messages, then activate OFFBOARD mode, to take effect
        # k=0
        # self.controller_00.sp.position.z = 3
        # while k<10:
        #    self.pub_position_00.publish(self.controller_00.sp)
        #    rospy.sleep(0.1)
        #    k = k + 1

        # activate OFFBOARD mode
        # self.modes_00.setOffboardMode()


def main():
    
    # initiate node
    rospy.init_node('off_board_and_arming_node', anonymous=True)

    # flight mode object
    modes_00 = fcuModes_00()
    modes_01 = fcuModes_01()
    modes_02 = fcuModes_02()
        
    # controller object
    cnt_00 = Controller()
    cnt_01 = Controller()
    cnt_02 = Controller()
    
    board = off_board(modes_00, modes_01, modes_02, cnt_00, cnt_01, cnt_02)
    
    # Make sure the drone is armed
    while not board.controller_00.state.armed:
        board.modes_00.setArm()
        board.rate.sleep()
    
    while not board.controller_01.state.armed:
        board.modes_01.setArm()
        board.rate.sleep()
        
    while not board.controller_02.state.armed:
        board.modes_02.setArm()
        board.rate.sleep()

    # We need to send few setpoint messages, then activate OFFBOARD mode, to take effect
    k=0
        
    while k<10:
        board.pub_position_00.publish(board.controller_00.sp)
        board.pub_position_01.publish(board.controller_01.sp)
        board.pub_position_02.publish(board.controller_02.sp)
        
        board.rate.sleep()
        k = k + 1

    # activate OFFBOARD mode
    board.modes_00.setOffboardMode()
    board.modes_01.setOffboardMode()
    board.modes_02.setOffboardMode()
    
    while (not rospy.is_shutdown()):
    # updateSp function renew the target position of drone with 20Hz   
        board.controller_00.updateSp()                # this function can maintain altitude of drone
        board.controller_01.updateSp()                # msg.pose.position = catch the drone of current position
        board.controller_02.updateSp()                # sp.position = target position of drone
        
        # publish the target posion of drone to Controller code for controling the drone position using this while loop
        board.pub_position_00.publish(board.controller_00.sp)   # sp is PositionTarget()
        board.pub_position_01.publish(board.controller_01.sp)
        board.pub_position_02.publish(board.controller_02.sp)
        
        while board.reset == True:
            if board.reset == False:
                break
            pass
        
        board.rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass