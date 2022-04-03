. CAPSTONE

Pycharm Terminal 수정 방법 (아나콘다와 연동 하기 위해서)

anaconda3 prompt 바로가기 속성 -> 대상 -> cmd.exe "/K" C:\Users\ejdjc\anaconda3\Scripts\activate.bat C:\Users\ejdjc\anaconda3

와 같이 설정해주면 Pycharm에서 바로 Anaconda Prompt가 실행가능하다.

Jetson Nano Torchvision 및 Pytorch

https://forums.developer.nvidia.com/t/pytorch-for-jetson-version-1-10-now-available/72048

https://github.com/kookmin-sw/2019-cap1-2019_6/blob/master/src/Final/final_beEyes_objectDetection.py 참고

python detect.py --source 0 --weights best.pt 

python detect.py --source st/ --weights best.pt --conf 0.4

python train.py --img 640 --batch 4 --epochs 5 --data data/coco128.yaml --cfg models/yolov5s.yaml --weights weights/yolov5s.pt


camera check

sudo apt-get install v4l-utils
v4l2-ctl --list-devices

torch with torchvision 0.9.0 error whith PIL 

pip install pillow==4.1.1 is worked I don't know why

not save mode

pip install -r requirements.txt
