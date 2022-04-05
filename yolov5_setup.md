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



yolov5 설치 

```markdown
git clone https://github.com/ultralytics/yolov5.git
```

을 통하여 yolov5 폴더가 생성된다.

후에 

```markdown
cd yolov5 #yolov5 폴더로 이동
wget https://github.com/ultralytics/yolov5/releases/download/v6.0/yolov5s.pt
```

yolov5 폴더로 이동하여 wget명령어를 통하여 5s 모델을 다운받는다. 

5n의 가벼운 모델도 나왔으니 무거울 경우 다운로드 하기



이후 yolov5 폴더에서

```mark
python3 -m pip install -r requirements.txt
```

를 하면 모든 요구사항들이 한번에 다운로드된다!



```markdown
python3 detect.py --source 0 #웹캠으로 yolov5 돌려보기
python3 detect.py --source ./data/images #제공해주는 사진으로 돌려보기
```

다음과 같은 예제가 잘 실행된다면 yolo의 설치까지 완성이다.
