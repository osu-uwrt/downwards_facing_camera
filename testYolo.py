import cv2
import numpy as np
from ultralytics import YOLO

image = cv2.imread("/home/markc/yolo_im.bmp")
# print(image.shape)
# image = np.zeros(image.shape)

model = YOLO('/home/markc/Downloads/Buoy_Gate_Torpedo_300_Rotation360_2575_Mirror_Nano.pt')

model.export(format="edgetpu")

# ret = model(image, conf=0.8, iou=0.0)

# print(len(ret))