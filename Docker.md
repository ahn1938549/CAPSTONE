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
docker run  docker run -p 6080:80 --name Cap ubuntu
```

-e 는 환경변수
-p 는 포트 지정으로 SSH 기능 사용
--name 사용자 네임으로 생성 (안하면 이름 이상해서 빡침)

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

근데 VCL에서 통신 못해서 못 함

```
docker run -p 6080:80 -p 5900:5900 -v /dev/shm:/dev/shm dorowu/ubuntu-desktop-lxde-vnc
```
으로 docker 이미지 킬 수 있으며 이는
웹에서 127.0.0.1:6080 으로 접속
혹은 127.0.0.1:5800 VNC server에서 접속
-it 붙이고 뒤에 bash 붙이면 bash로 터미널 접속


https://lovemewithoutall.github.io/it/ubuntu-vnc-desktop/

/bin/bash 를 킨 후

```rpm -qa | more```
