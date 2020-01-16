## 2. 실행

'''
1. roscore
2. make px4_sitl_default gazebo
3. roslaunch modudculab_ros ctrl_pos_gazebo.launch fcu_url="udp://:14540@IP:14557"
4. rosrun mavros mavsafety
5. rosrun mavros mavsys mode -c OFFBOARD
'''

