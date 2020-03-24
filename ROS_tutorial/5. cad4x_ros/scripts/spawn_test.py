#!/usr/bin/env python
import rospy
import rospkg
import roslaunch
import subprocess
import os

from gazebo_msgs.srv import DeleteModel, SpawnModel # For deleting models from the environment

#uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
#roslaunch.configure_logging(uuid)
#launch = roslaunch.parent.ROSLaunchParent(uuid, [])

def del_model(): # FIXME: Freezes Python, DO NOT USE!
    package = 'px4'
    launch_file = 'multi_uav.launch'
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/parallels/catkin_ws/src/Firmware/launch/multi_uav.launch"])
    
    """ Remove the model with 'modelName' from the Gazebo scene """
    
    # delete_model : gazebo_msgs/DeleteModel
    del_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel) # model spawner
    # spawn_model = rospy.ServiceProxy('gazebo/spawn_model', SpawnModel)
    # rospy.wait_for_service('gazebo/delete_model') # Wait for the model loader to be ready 
    # FREEZES EITHER WAY
    modelName_1 = "iris_0"
    modelName_2 = "iris_1"
    modelName_3 = "iris_2"
    del_model_prox(modelName_1) # Remove from Gazebo
    del_model_prox(modelName_2) # Remove from Gazebo
    del_model_prox(modelName_3) # Remove from Gazebo
    
    launch.start()
    rospy.sleep(3)
    
    #launch.start()
    
if __name__ == '__main__':
    try:
		del_model()
    except rospy.ROSInterruptException:
		pass