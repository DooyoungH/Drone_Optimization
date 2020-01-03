# ROS (Robot Operating System)

## 1. ROS 
ROS는 로봇산업분야에서 많이 사용되고 있는 툴로써 로봇을 URDF (Universal Robotic Description Format) 로 작성하고 이 로봇과 환경에 여러 외부 요소 (관성, 속도, 가속도 같은 힘과 물체의 특성치) 를 고려하여 로봇의 움직임을 시뮬레이션으로 확인해 볼 수 있습니다.

우리나라는 현재 표윤석 박사님을 필두로 운영되는 오픈 로봇 기술 공유 카페 [오로카] [1]를 필두로 ROS 에 대한 연구들이 진행되고 있으며, 해당 카페에서 어느정도의 튜토리얼은 수월하게 진행 할 수 있습니다. 

하지만 응용부분에 넘어선다면 한글화 자료가 매우 찾기 힘들며, ROS의 기능이 활용되는 프로젝트를 직접 수행해 보지 않는다면 자신이 원하는 환경에 맞추어서 패키지를 변경하기도 매우 쉽지 않습니다. 
따라서 본 문서에서는 어느 정도 ROS 에 쉽게 적응할 수 있게끔 로봇의 설계와 토픽의 전송을 통한 로봇의 움직임 설정,
그리고 최종적으로 PX4 를 이용한 드론의 자율주행 시뮬레이션 환경 구축을 진행하고자 합니다.

## 2. 환경 설정
기본적으로 ROS는 Linux 환경을 사용합니다. 현재 제가 사용하는 환경은 Ubuntu 16.04 LTS 와 ROS Kinetic 을 사용하며, ROS kinetic 의 경우 오로카에서 표윤석 박사님이 작성하신 한 줄 설치 코드로 쉽게 설치 할 수 있습니다.

'''
wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_kinetic.sh && chmod 755 ./install_ros_kinetic.sh && bash ./install_ros_kinetic.sh
'''

Onboard computer (라즈베리파이) 의 경우 는 다음의 코드로 한 줄 설치가 가능합니다.

'''
wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_kinetic_rp3.sh && chmod 755 ./install_ros_kinetic_rp3.sh && bash ./install_ros_kinetic_rp3.sh
'''

[출처] ROS Kinetic 1줄 설치! (오픈소스 소프트웨어 & 하드웨어: 로봇 기술 공유 카페 (오로카)) |작성자 표윤석


## 3. 


해당 폴더에 포함된 내용은 다음과 같습니다.
* RTOS 를 이용한 임베디드 보드의 전력 소모 측정 코드
  * 우리는 TI 사의 MCU 를 토대로 하는 임베디드 보드를 제작하였으며, 해당 보드는 매우 높은 정확도와 1 kHz의 샘플링레이트를 가집니다.
  * 문서로 남겨 놓은 사항은 TI 사의 IDE인 CCS의 세팅과 작성한 코드에 대한 설명으로, 코드가 동작하는 플로우와 각 함수의 역할을 설명하고자 합니다. 






[1] : https://cafe.naver.com/openrt 
 