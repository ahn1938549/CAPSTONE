. CAPSTONE

2022년 캡스톤 프로젝트 

목표 : Object Detection 기반의 계단 인식  

준비물 : Jetson Nano 



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



yolov5 사용 on Jetson Nano	

https://forums.developer.nvidia.com/t/pytorch-for-jetson-version-1-10-now-available/72048

을 참고



```markdown
sudo apt-get install python3-pip libopenblas-base libopenmpi-dev
sudo pip3 install Cython
sudo apt-get install libjpeg-dev zlib1g-dev libpython3-dev libavcodec-dev libavformat-dev libswscale-dev
```

으로 초기에 필요한 모듈들을 전부 설치



```markdown
wget https://nvidia.box.com/shared/static/p57jwntv436lfrd78inwl7iml6p13fzh.whl -O torch-1.8.0-cp36-cp36m-linux_aarch64.whl
```

1.8.0 버전 다운로드 (버전은 입맛에 맞게)



```markdown
pip3 install numpy torch-1.8.0-cp36-cp36m-linux_aarch64.whl
```

1.8.0 버전 설치 

python3 실행 후

```markdown
>>> import torch
>>> print(torch.__version__)
```

에서 에러가 없고 버전이 잘 뜰 경우 설치 완료



torchvision 설치

```markdown
git clone --branch v0.9.0 https://github.com/pytorch/vision torchvision
```

완료 시에

```markdown
cd torchvision
export BUILD_VERSION=0.9.0
python3 setup.py install --user
```

후에 python 실행 후

```markdown
>>> import torchvision
>>> print(torchvision.__version__)
```

에서 잘 뜬다면 설치 성공

```markdown
torch with torchvision 0.9.0 error whith PIL
```

에러는 

```markdown
pip install pillow==4.1.1
```

설치시에 해결 가능하다.


Pycharm Terminal 수정 방법 (아나콘다와 연동 하기 위해서)

anaconda3 prompt 바로가기 속성 -> 대상 -> cmd.exe "/K" C:\Users\ejdjc\anaconda3\Scripts\activate.bat C:\Users\ejdjc\anaconda3

와 같이 설정해주면 Pycharm에서 바로 Anaconda Prompt가 실행가능하다.



https://github.com/kookmin-sw/2019-cap1-2019_6/blob/master/src/Final/final_beEyes_objectDetection.py 참고

python detect.py --source 0 --weights best.pt 

python detect.py --source st/ --weights best.pt --conf 0.4

python train.py --img 640 --batch 4 --epochs 5 --data data/coco128.yaml --cfg models/yolov5s.yaml --weights weights/yolov5s.pt


pip install -r requirements.txt

python3 detect.py --soucre 0 "nvarguscamerasrc ! video/x-raw(memory:NVMM),width=1280, height=720, framerate=30/1, format=NV12 ! nvvidconv flip_method=0 ! video/x-raw, format=BGRx, width=640, height=480 ! videoconvert ! video/x-raw, format=BGR ! appsink"

fps를 높이려고 찾아본 결과 protobuf와 deepstream을 찾을 수 있었고 nano5s 모델도 있는 것을 확인하였다. 22-04-04

TensorRT를 위해선 ONNX가 필요
ONNX설치중 Protobuf를 필요로한다.

deepstream은 TensorRt가 필요로 한다.

Deepstream을 실행시키기 위해 TensorRT
TensorRT를 위해 ONNX
ONNX를 위해 Protobuf

Docker 설치하기
