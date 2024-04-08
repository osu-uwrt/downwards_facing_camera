import cv2
import numpy as np
from ultralytics import YOLO

image = cv2.imread("/home/markc/yolo_im.bmp")
# print(image.shape)
# image = np.zeros(image.shape)

model = YOLO('/home/markc/Downloads/bin300.pt')

model.export(format="onnx")

# ret = model(image, conf=0.8, iou=0.0)

# print(len(ret))