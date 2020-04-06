#!/usr/bin/env python
# ROS python API
import rospy
import math

# 3D point & Stamped Pose msgs
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, PoseStamped, Quaternion, QuaternionStamped
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
        self.ps = PoseStamped()

        # set the flag to use position setpoints and yaw angle
        self.sp.type_mask = 3064
        #self.sp.type_mask = int('010111111000', 2)
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

        self.clbk_yaw = 2.3561
        self.clbk_yaw_rate = 0.0

        # A Message for the current local position of the drone
        self.local_pos = Point(0.0, 0.0, 0)
        self.local_ori = Quaternion(0.0, 0.0, 0.0, 0.0)

        # initial values for setpoints
        self.sp.position.x = 0.0
        self.sp.position.y = 0.0
        
        self.sp.velocity.x = 0.0
        self.sp.velocity.y = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        # speed of the drone is set using MPC_XY_CRUISE parameter in MAVLink
        # using QGroundControl. By default it is 5 m/s.

	# Callbacks

    ## local position callback
    def posCb(self, msg):
        self.local_pos.x = msg.pose.position.x
        self.local_pos.y = msg.pose.position.y
        self.local_pos.z = msg.pose.position.z

    def oriCb(self, msg):
        self.local_ori.x = msg.pose.orientation.x
        self.local_ori.y = msg.pose.orientation.y
        self.local_ori.z = msg.pose.orientation.z
        self.local_ori.w = msg.pose.orientation.w

    ## Drone State callback
    def stateCb(self, msg):
        self.state = msg

    ## Update setpoint message
    def updateSp(self):
        self.sp.position.x = self.clbk_position_x
        self.sp.position.y = self.clbk_position_y
        self.sp.position.z = self.clbk_position_z
        
        # control the velocity of drones
        self.sp.velocity.x = self.clbk_velocity_x
        self.sp.velocity.y = self.clbk_velocity_y
       
        # control the yaw of drones
        self.sp.yaw = self.clbk_yaw
        self.sp.yaw_rate = self.clbk_yaw_rate
        

    def callback(self, msg):
        self.clbk_position_x = msg.position.x
        self.clbk_position_y = msg.position.y
        self.clbk_position_z = msg.position.z
        
        self.clbk_velocity_x = msg.velocity.x
        self.clbk_velocity_y = msg.velocity.y
        self.clbk_velocity_z = msg.velocity.z

        self.clbk_yaw = msg.yaw
        self.clbk_yaw_rate = msg.yaw_rate

    def x_dir(self):
    	self.sp.position.x = self.local_pos.x + 5
    	self.sp.position.y = self.local_pos.y

        self.sp.yaw = math.radians(0)
        self.sp.yaw_rate = 0.0
     
    def neg_x_dir(self):
    	self.sp.position.x = self.local_pos.x - 5
    	self.sp.position.y = self.local_pos.y

        self.sp.yaw = math.radians(180)
        self.sp.yaw_rate = 0.0

    def y_dir(self):
    	self.sp.position.x = self.local_pos.x
    	self.sp.position.y = self.local_pos.y + 5

        self.sp.yaw = math.radians(90)
        self.sp.yaw_rate = 0.0

    def neg_y_dir(self):
    	self.sp.position.x = self.local_pos.x
    	self.sp.position.y = self.local_pos.y - 5

        self.sp.yaw = math.radians(-90)
        self.sp.yaw_rate = 0.0

    def up(self):
        self.sp.position.x = self.local_pos.x
        self.sp.position.y = self.local_pos.y
        self.sp.position.z = self.local_pos.z + 5
    
    # positive x + positive y + yaw 45
    def action_4(self):
        self.sp.position.x = self.local_pos.x + 5
        self.sp.position.y = self.local_pos.y + 5

        self.sp.yaw = math.radians(45)
        self.sp.yaw_rate = 0.0       
    
    # positive x + negative y + yaw -45
    def action_5(self):
        self.sp.position.x = self.local_pos.x + 5
        self.sp.position.y = self.local_pos.y - 5

        self.sp.yaw = math.radians(-45)
        self.sp.yaw_rate = 0.0
    
    # negative x + positive y + yaw 135
    def action_6(self):
        self.sp.position.x = self.local_pos.x - 5
        self.sp.position.y = self.local_pos.y + 5

        self.sp.yaw = math.radians(135)
        self.sp.yaw_rate = 0.0
    
    # negative x + negative y + yaw -135    
    def action_7(self):
        self.sp.position.x = self.local_pos.x - 5
        self.sp.position.y = self.local_pos.y - 5

        self.sp.yaw = math.radians(-135)
        self.sp.yaw_rate = 0.0
    
    # up + positive x + yaw 0
    def action_10(self):
        self.sp.position.x = self.local_pos.x + 5
        self.sp.position.z = self.local_pos.z + 5

        self.sp.yaw = math.radians(0)
        self.sp.yaw_rate = 0.0
    
    # up + positive y + yaw 90
    def action_11(self):
        self.sp.position.y = self.local_pos.y + 5
        self.sp.position.z = self.local_pos.z + 5
    
        self.sp.yaw = math.radians(90)
        self.sp.yaw_rate = 0.0

    # up + negative x + yaw 180
    def action_12(self):
        self.sp.position.x = self.local_pos.x - 5
        self.sp.position.z = self.local_pos.z + 5

        self.sp.yaw = math.radians(180)
        self.sp.yaw_rate = 0.0
    
    # up + negative y + yaw -90
    def action_13(self):
        self.sp.position.y = self.local_pos.y - 5
        self.sp.position.z = self.local_pos.z + 5

        self.sp.yaw = math.radians(-90)
        self.sp.yaw_rate = 0.0
    
    # up + positive x + positive y + yaw 45
    def action_14(self):
        self.sp.position.x = self.local_pos.x + 5
        self.sp.position.y = self.local_pos.y + 5
        self.sp.position.z = self.local_pos.z + 5

        self.sp.yaw = math.radians(45)
        self.sp.yaw_rate = 0.0
    
    # up + positive x + negative y + yaw -45
    def action_15(self):
        self.sp.position.x = self.local_pos.x + 5
        self.sp.position.y = self.local_pos.y - 5
        self.sp.position.z = self.local_pos.z + 5

        self.sp.yaw = math.radians(-45)
        self.sp.yaw_rate = 0.0
    
    # up + negative x + positive y + yaw 135
    def action_16(self):
        self.sp.position.x = self.local_pos.x - 5
        self.sp.position.y = self.local_pos.y + 5
        self.sp.position.z = self.local_pos.z + 5

        self.sp.yaw = math.radians(135)
        self.sp.yaw_rate = 0.0
    
    # up + negative x + negative y + yaw -135
    def action_17(self):
        self.sp.position.x = self.local_pos.x - 5
        self.sp.position.y = self.local_pos.y - 5
        self.sp.position.z = self.local_pos.z + 5

        self.sp.yaw = math.radians(-135)
        self.sp.yaw_rate = 0.0
    
    # down + positive x
    def action_18(self):
        self.sp.position.x = self.local_pos.x + 5
        self.sp.position.z = self.local_pos.z - 5
    
    # down + positive y
    def action_19(self):
        self.sp.position.y = self.local_pos.y + 5
        self.sp.position.z = self.local_pos.z - 5
    
    # down + negative x
    def action_20(self):
        self.sp.position.x = self.local_pos.x - 5
        self.sp.position.z = self.local_pos.z - 5
    
    # down + negative y
    def action_21(self):
        self.sp.position.y = self.local_pos.y - 5
        self.sp.position.z = self.local_pos.z - 5

    # down + positive x + positive y
    def action_22(self):
        self.sp.position.x = self.local_pos.x + 5
        self.sp.position.y = self.local_pos.y + 5
        self.sp.position.z = self.local_pos.z - 5
    
    # down + positive x + negative y
    def action_23(self):
        self.sp.position.x = self.local_pos.x + 5
        self.sp.position.y = self.local_pos.y - 5
        self.sp.position.z = self.local_pos.z - 5
    
    # down + negative x + positive y
    def action_24(self):
        self.sp.position.x = self.local_pos.x - 5
        self.sp.position.y = self.local_pos.y + 5
        self.sp.position.z = self.local_pos.z - 5
    
    # down + negative x + negative y 
    def action_25(self):
        self.sp.position.x = self.local_pos.x - 5
        self.sp.position.y = self.local_pos.y - 5
        self.sp.position.z = self.local_pos.z - 5

    def down(self):
        self.sp.position.x = self.local_pos.x
        self.sp.position.y = self.local_pos.y
        self.sp.position.z = self.local_pos.z - 5
     
    def stop(self):
        self.sp.position.x = self.local_pos.x
        self.sp.position.y = self.local_pos.y
        self.sp.position.z = self.local_pos.z

    def origin(self):
        self.sp.position.x = 0.0
        self.sp.position.y = 0.0
        self.sp.position.z = 3.0

        self.sp.velocity.x = 8.0
        self.sp.velocity.y = 8.0
        self.sp.velocity.z = 0.5

    def origin_00(self):
        self.sp.position.x = 0.0
        self.sp.position.y = 0.0
        self.sp.position.z = 3.0

    def origin_01(self):
        self.sp.position.x = 0.0
        self.sp.position.y = 0.0
        self.sp.position.z = 3.0
            
    def origin_02(self):
        self.sp.position.x = 0.0
        self.sp.position.y = 0.0
        self.sp.position.z = 3.0

    def quaternion_to_euler(self):
        quaternion = (
            self.local_ori.w,
            self.local_ori.x,
            self.local_ori.y,
            self.local_ori.z)
        
        euler = euler_from_quaternion(quaternion, axes="sxyz")

        self.yaw = euler[2]


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
        
        self.sub_orientation_00 = rospy.Subscriber('/uav0/mavros/local_position/pose', QuaternionStamped, self.controller_00.oriCb)

        # Subscribe to drone's velocity
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
        rospy.wait_for_service('/gazebo/reset_world')
        self.reset_simulation = rospy.ServiceProxy('/gazebo/reset_world', Empty)

        # ROS loop rate
        self.rate = rospy.Rate(20.0)
        
    def reset_callback(self, msg):
        self.reset = msg.data

    # Make sure the drone is armed
    def set_arming(self):     
        while not self.controller_00.state.armed:
            self.modes_00.setArm()
            self.rate.sleep()
    
        while not self.controller_01.state.armed:
            self.modes_01.setArm()
            self.rate.sleep()
        
        while not self.controller_02.state.armed:
            self.modes_02.setArm()
            self.rate.sleep()
    
    def set_offboard(self): 
    # We need to send few setpoint messages, then activate OFFBOARD mode, to take effect
        k=0
        while k<10:
            self.pub_position_00.publish(self.controller_00.sp)
            self.pub_position_01.publish(self.controller_01.sp)
            self.pub_position_02.publish(self.controller_02.sp)
        
            self.rate.sleep()
            k = k + 1

        # activate OFFBOARD mode
        self.modes_00.setOffboardMode()
        self.modes_01.setOffboardMode()
        self.modes_02.setOffboardMode()

    # updateSp function renew the target position of drone with 20Hz
    def position_update(self):
        self.controller_00.updateSp()                # this function can maintain altitude of drone
        self.controller_01.updateSp()                # msg.pose.position = catch the drone of current position
        self.controller_02.updateSp()                # sp.position = target position of drone
        
        # publish the target posion of drone to Controller code for controling the drone position using this while loop
        self.pub_position_00.publish(self.controller_00.sp)   # sp is PositionTarget()
        self.pub_position_01.publish(self.controller_01.sp)
        self.pub_position_02.publish(self.controller_02.sp)


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
    
    board.set_arming()
    board.set_offboard()

    stop_count = 0

    while (not rospy.is_shutdown()):
        board.position_update()
        board.rate.sleep()
        board.controller_00.quaternion_to_euler()
        while (not rospy.is_shutdown()) and (board.reset == True):
            # 1. return the true value of board.reset from the action.py
            # 2. stop the all drones 
            board.controller_00.stop()
            board.controller_01.stop()
            board.controller_02.stop()

            board.position_update()
            board.rate.sleep()

            stop_count += 1
            if (stop_count == 10):
                print('working?')
                while(not rospy.is_shutdown()):
                
                    board.position_update()
                    board.rate.sleep()
                    
                    if board.reset == False:
                        print("reset pending resume")
                        break

                stop_count = 0


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass