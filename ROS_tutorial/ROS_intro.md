# ROS (Robot Operating System)

## 1. ROS 
ROS는 로봇산업분야에서 많이 사용되고 있는 툴로써 로봇을 URDF (Universal Robotic Description Format) 로 작성하고 이 로봇과 환경에 여러 외부 요소 (관성, 속도, 가속도 같은 힘과 물체의 특성치) 를 고려하여 로봇의 움직임을 시뮬레이션으로 확인해 볼 수 있습니다.

우리나라는 현재 표윤석 박사님을 필두로 운영되는 오픈 로봇 기술 공유 카페 [오로카] [1]를 필두로 ROS 에 대한 연구들이 진행되고 있으며, 해당 카페에서 어느정도의 튜토리얼은 수월하게 진행 할 수 있습니다. 

하지만 응용부분에 넘어선다면 한글화 자료가 매우 찾기 힘들며, ROS의 기능이 활용되는 프로젝트를 직접 수행해 보지 않는다면 자신이 원하는 환경에 맞추어서 패키지를 변경하기도 매우 쉽지 않습니다. 
따라서 본 문서에서는 어느 정도 ROS 에 쉽게 적응할 수 있게끔 로봇의 설계와 토픽의 전송을 통한 로봇의 움직임 설정,
그리고 최종적으로 PX4 를 이용한 드론의 자율주행 시뮬레이션 환경 구축을 진행하고자 합니다.

## 2. 환경 설정
기본적으로 ROS는 Linux 환경을 사용합니다. 현재 제가 사용하는 환경은 Ubuntu 16.04 LTS 와 ROS Kinetic 을 사용하며, ROS kinetic 의 경우 오로카에서 표윤석 박사님이 작성하신 한 줄 설치 코드로 쉽게 설치 할 수 있습니다.

```
wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_kinetic.sh && chmod 755 ./install_ros_kinetic.sh && bash ./install_ros_kinetic.sh
```

Onboard computer (라즈베리파이) 의 경우 는 다음의 코드로 한 줄 설치가 가능합니다.

```
wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_kinetic_rp3.sh && chmod 755 ./install_ros_kinetic_rp3.sh && bash ./install_ros_kinetic_rp3.sh
```

[출처] ROS Kinetic 1줄 설치! (오픈소스 소프트웨어 & 하드웨어: 로봇 기술 공유 카페 (오로카)) |작성자 표윤석


## 3. IDE

```

```


## 4. TEST RUN
ROS 가 설치 되었다면 설치가 제대로 완료되었는지 기본적으로 테스트가 필요합니다.




## 5. 내용 구성
Turtlesim 이 제대로 돌아간다면 기본적인 세팅은 완료 된 것입니다. 
ROS 를 제대로 활용하기 위하여 앞으로 진행 할 내용은 다음과 같습니다.

* ROS 의 기본적인 설계 개념과 퍼블리셔 및 서브스크라이버 제작해보기 (roscpp, rospy)
* URDF 를 이용한 Two wheel robot 제작 및 rospy 를 통해서 로봇을 움직여보기
* Gazobo 를 이용한 환경 구성 및 Two wheel robot 에 환경 적용해보기
* Two wheel robot 과 센서를 이용한 maze 탈출 코드 구현 및 적용해보기
* PX4 시뮬레이션 환경 구현
* 드론의 비행 환경 Gazebo 로 구현해보기
* 강화학습을 통한 드론의 최적화 비행 구현  



[1] : https://cafe.naver.com/openrt 
 