. CAPSTONE

2022년 캡스톤 프로젝트 

목표 : Object Detection 기반의 계단 인식  

준비물 : Jetson Nano 




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
