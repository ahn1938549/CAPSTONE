# Yolov5 고찰

Jetson Nano에서 Yolov5를 돌릴경우 130ms 즉 7 FPS를 확인 할 수 있다. 이는 생각보다 느린 속도이다.



FPS를 높이려고 찾아본 결과 protobuf와 deepstream을 찾을 수 있었고 

yolov5 github에서 업데이트된 nano5s 모델도 있는 것을 확인하였다. 

Deepstream을 실행시키기 위해 TensorRT
TensorRT를 위해 ONNX
ONNX를 위해 Protobuf가 필요하며 



이는 여태까지 써온 torch와는 다른 Tensor를 사용한다.



**Yolo의 속도를 높이기 위하여 다음과 같이 실험하려고 한다.**

1. Docker 설치
2. Protobuf 설치
3. ONNX 설치
4.  TensorRT 설치
5. Deepstream 설치

도중에 삑사리 안나기를 기도하자.
