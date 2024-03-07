import cv2
import numpy as np

import csv

bboxes = []
classes = []
masks = []

with open('/home/markc/Detections.csv') as file:
    reader = csv.reader(file)
    for lines in reader:
        bboxes.append([float(lines[0]) * 320, float(lines[1]) * 320, float(lines[2]) * 320, float(lines[3]) * 320])
        classes.append(int(lines[4]) + 127)
        
with open('/home/markc/masks.csv') as file:
    reader = csv.reader(file)
    for lines in reader:
        mask = np.ndarray((6400, 1))
        for i in range(6400):
            mask[i] = float(lines[i])
        masks.append(mask)

image = cv2.imread("/home/markc/template_small.bmp")
for i in range(len(bboxes)):
    imageCpy = image.copy()
    mask = masks[i]
    # print(mask.shape)
    mask = mask.astype(np.uint8)
    mask *= 255
    # bboxSize = (bboxes[i][3], bboxes[i][2])
    mask = mask.reshape((80, 80), order='F')
    mask = cv2.resize(mask, (320, 320))
    print(mask.shape)
    cv2.imshow("test", mask.T)
    cv2.waitKey(0)
    # imageCpy = np.where(mask[..., None], (0, 255, 0), image)
    # image = cv2.addWeighted(image, 0.8, imageCpy, 0.2, 0)
    # print(bboxes[i][0])
    # print((bboxes[i][0] - bboxes[i][2] / 2, bboxes[i][1] - bboxes[i][3] / 2))
    # cv2.rectangle(image, (int(bboxes[i][0] - bboxes[i][2] / 2), int(bboxes[i][1] - bboxes[i][3] / 2)), (int(bboxes[i][0] + bboxes[i][2] / 2), int(bboxes[i][1] + bboxes[i][3] / 2)), (0, 255, 0))
    
    
cv2.imshow("Image", image)
cv2.waitKey(0)