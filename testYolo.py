import cv2
import numpy as np
from ultralytics import YOLO

image = cv2.imread("/home/markc/yolo_im.bmp")
# print(image.shape)
image = np.zeros(image.shape)

model = YOLO('yolov8n-seg.pt')

# model.export(format="onnx")

ret = model(image, conf=0.8)

# print(len(ret))