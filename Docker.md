설치는 쉽다. 알아서 하고 초기 상태로 오자!

터미널에서 ```markdown docker pull ubuntu:20.04 ``` 과 같이 칠 경우

Docker 에서 다음과 같이 보여질 것이다.

![초기 설정](images/fir.png)

다음과 같은 상태에서

```
docker run  -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=unix$DISPLAY --name Cap ubuntu
```

X11서버를 사용하여 공유

```
docker run  docker run -p 6080:80 --it --name Cap ubuntu
```

-e 는 환경변수
-p 는 포트 지정으로 SSH 기능 사용
--name 사용자 네임으로 생성 (안하면 이름 이상해서 빡침)



```
docker run -p 6080:80 -p 5900:5900 --it -v /dev/shm:/dev/shm dorowu/ubuntu-desktop-lxde-vnc
```
으로 docker 이미지 킬 수 있으며 이는   
웹에서 127.0.0.1:6080 으로 접속   
혹은 127.0.0.1:5900 VNC server에서 접속   
-it 붙이고 뒤에 bash 붙이면 bash로 터미널 접속   

현재 웹으로 접속하고 있으며 Ros_noletic버전을 설치하여 사용하고 있다.

![현재상황](/home/bong/capston/CAPSTONE/images/Docker_images.png)

gui_cap은 현재 돌아가고 있는 원격 컨테이너이며   
no_wifi는 gui를 볼 수 없어서 포기한 것이다.    
나머지는 나도 모른다. 왜 vnc가 두개일까?

https://lovemewithoutall.github.io/it/ubuntu-vnc-desktop/

/bin/bash 를 킨 후

```rpm -qa | more```

```
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

source ~/.bashrc

sudo apt-get install python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

mkdir -p catkin_ws/src

cd catkin_ws/

catkin_make
```
# 망한 것
GUI를 위하여 Docker안에서 VNC 설치
```
sudo apt install tigervnc-standalone-server tigervnc-common tigervnc-xorg-extension tigervnc-viewer
```

```
#!/bin/sh
# Start Gnome 3 Desktop
[ -x /etc/vnc/xstartup ] && exec /etc/vnc/xstartup
[ -r $HOME/.Xresources ] && xrdb $HOME/.Xresources
vncconfig -iconic &
dbus-launch --exit-with-session gnome-session &
```
후에 우분투에 GUI가 없다고 판단 우분투 GUI버전을 직접 인스톨하였고 다음과 같은 코드로 실행하였으나 GUI를 결국 못 보면서 실패...
```
startx
```
근데 VCL에서 통신 못해서 못 함
