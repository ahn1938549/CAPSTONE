# Ros 정리

Ros는 전부 Docker안에서 구동하였다.   


```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

을 통하여 noetic을 설치하였다.


```
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```
외부 키를 안전하게 가져왔으며

```
sudo apt update
```
으로 펌웨어를 업데이트 하였다.

```
sudo apt install ros-noetic-desktop-full
```
이거 깔았다. 참고로 desktop-full 버전은 뉴비용 잡다한 거 다 넣어준거
base는 베어본용(GUI 없다)

```
sudo apt-get install python-pip
sudo pip install -U rosdep
```
으로 필요한 걸 전부 깔고

```
sudo rosdep init
```
초기화를 한 번 실행하여준다

```
rosdep update #sudo 쓰기 금지
```
sudo 쓰지말고 하랜다. (권한 문제가 추후 발생)

```
mkdir -p ~/catkin_ws/src
```
```
cd ~/catkin_ws/src
```
```
catkin_init_workspace
```

순차적으로 실행할 경우 catkin ROS 전용 빌드 시스템을 초기화하여 사용할 수 있다.
```
cd
catkin_make
```
로 빌드 할 수 있다.

# catkin 환경 파일 설정
```
gedit ~/.bashrc #nano나 vi 가능
```
맨 밑에 붙여 넣어 주기
```
alias eb =‘nano ~/.bashrc'
alias sb ='source ~/.bashrc'
alias cw ='cd ~/catkin_ws'
alias cs ='cd ~/catkin_ws/src'
alias cm ='cd ~/catkin_ws && catkin_make'
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
#export ROS_MASTER_URI=http://192.168.1.100:11311
#export ROS_HOSTNAME=192.168.1.100
```

끝

# 예제

```
roscore
```

```
rosrun turtlesim turtlesim_node
```

```
rosrun turtlesim turtle_teleop_key
```

각 각의 터미널에서 실행하기
