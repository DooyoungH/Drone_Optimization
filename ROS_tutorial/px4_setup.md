
## 1. 설치

초기 설치 셋업은 모두의연구소에서 이웅원님이 작성하신 글 [1]을 참고하였습니다.

다만 이 설치 방법은 Ubuntu 14.04 를 토대로 진행하고 있으며, Ubuntu 16.04 에서 설치 및 실행하기 위한 라이브러리들과
설치 중 생겼던 오류들을 해결하기 위한 기타 라이브러리들이 추가적으로 기재하고자 다음의 글을 작성하였습니다.


우선 apt-get 을 업데이트
mavros package 를 설치
pixhawk4 (PX4) 를 설치 전 의존성 해결을 위한 라이브러리 설치
openjdk-8-jdk, open-8-jre 는 Java 이며, Ubuntu 14.04 버전을 사용하는 경우 openjdk-7-jdk, open-7-jre 을 설치하여야 함 

pixhawk4 설치

```
1. sudo apt-get install update
2. sudo apt-get install ros-kinetic-mavros ros-kinetic-mavros-extras
3. sudo add-apt-repository ppa:george-edison55/cmake-3.x -y
4. sudo apt-get update
5. sudo apt-get install python-argparse git-core wget zip python-empy qtcreator cmake build-essential genromfs -y 
6. sudo apt-get install ant protobuf-compiler libeigen3-dev libopencv-dev openjdk-8-jdk openjdk-8-jre clang-3.5 lldb-3.5 -y
7. java -version 커맨드를 통해 설치가 되었는지 확인
```

```
openjdk version "1.8.0_232"
OpenJDK Runtime Environment (build 1.8.0_232-8u232-b09-0ubuntu1-16.04.1.-b09)
OpenJDK 64-Bit Server VM (build 25.232-b09, mixed mode)
```

```
1. cd /catkin_ws/src
2. git clone https://github.com/PX4/Firmware.git
3. cd Firmware
4. git submodule update --init --recursive
```


모두의 연구소에서 진행한 방법은 iris drone 을 가제보에서 sitl 해보기 위하여 posix_sitl_default 라는 launch 파일을 실행하지만,
어째선지 Ubuntu 16.04 LTS 환경에서는 실행되지 않습니다.
따라서 px4 홈페이지에서 시행하는 방법을 통하여 gazebo 화면을 띄워보겠습니다. 

우선 추가적인 python 패키지가 필요합니다.
pip 가 설치 되지 않은 경우 다음의 명령어를 통해 pip를 설치합니다.

pip이란 python으로 작성된 패키지의 설치 및 관리를 해주는 프로그램으로, pip을 이용하면 의존성 문제를 자동적으로 해결해주기 때문에 편리합니다.

```
sudo apt-get install python3-pip
```

```
pip --version
pip 8.1.1 from /usr/lib/python2.7/dist-packages (python 2.7)
pip3 --version
pip 8.1.1 from /usr/lib/python3/dist-packages (python 3.5)
```

numpy 라이브러리와 toml 라이브러리 및 yaml 라이브러리 설치

```
sudo apt-get install python3-empy
sudo pip3 install numpy toml
sudo pip3 install pyyaml
```

이제 iris drone 을 gazebo 화면 상에 띄우기 위하여 다음과 같이 make 를 실행합니다.

```
1. cd /catkin_ws/src/Firmware
2. make px4_sitl_default gazebo
```

여기서 많은 분들이 에러가 발생한다고 하셨는데, 해외 사이트를 이래저래 찾아본 결과 GSTREAMER 라이브러리의 문제[2] 임을 확인하였습니다.


```
sudo apt-get install libgstreamer-plugins-base1.0-dev
```

설치 후 다시 실행해줍니다.


다음과 같이 화면이 출력된다면 기본적인 설치는 완료 된 것입니다.




## 2. 실행

```
1. roscore
2. make px4_sitl_default gazebo
3. roslaunch modudculab_ros ctrl_pos_gazebo.launch fcu_url="udp://:14540@IP:14557"
4. rosrun mavros mavsafety
5. rosrun mavros mavsys mode -c OFFBOARD
```





## 3. 다수 기종 실행

```
1. DONT_RUN=1 make px4_sitl_default gazebo
2. source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
3. export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd):$(pwd)/Tools/sitl_gazebo
4. roslaunch px4 multi_uav_mavros_sitl.launch
```




```
rosservice call /uav2/mavros/set_mode "base_mode: 0 custom_mode: 'OFFBOARD'"
rosservice call /uav2/mavros/cmd/arming "value: true"
'rostopic pub -r 10 /uav2/mavros/setpoint_raw/local '
```


[1] : http://www.modulabs.co.kr/index.php?mid=board_GDCH80&page=2&document_srl=1886
[2] : https://github.com/PX4/Firmware/issues/13117