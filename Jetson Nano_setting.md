Jetson Nano 준비 (SD카드 필수)

https://developer.nvidia.com/embedded/jetpack 에서 Jetpack 다운로드 

iso 프로그램을 이용하여 SD카드를 부팅용으로 설치

Jetson에 설치 및 apt update 및 apt upgrade  하기 (초기 안정화)



카메라 확인

```markdown
sudo apt-get install v4l-utils -y  
```

를 통하여 카메라 관련 설정들을 설치 한 후 

```markdown
v4l2-ctl --list-devices
```

입력하면 현재 카메라 리스트들과 카메라 지원 정보 등을 확인 할 수 있습니다.

```markdown
gst-launch-1.0 nvarguscamerasrc sensor_id=0 ! \
   'video/x-raw(memory:NVMM),width=1920, height=1080, framerate=30/1' ! \
   nvvidconv flip-method=0 ! 'video/x-raw,width=960, height=540' ! \
   nvvidconv ! nvegltransform ! nveglglessink -e
```

는 카메라 설정등을 통하여 카메라를 스크린에 불러오는 것인데 width와 height를 조절 가능합니다.





Jetson nano에 Anaconda 설치



https://github.com/conda-forge/miniforge 에서 

```markdown
wget "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-aarch64.sh"
```

후에

```markdown
bash Mambaforge-Linux-aarch64.sh
```

나오는 것 모두 y를 누르면

터미널 창 앞에 (base)가 뜬다. 



conda create -n (name) python==X.X.X : 가상환경 생성 명령어

conda activate (name)  : name 가상환경 활성화

conda deactivate : 가상환경 비활성화

conda env list : 가상환경 list 보기



초기 환경 설정 완료
