import numpy as np
import cv2
import math

import csv

# def createMask(mask: np.ndarray, bbox: tuple):
#     black = np.zeros((640, 640), np.uint8)
#     black[int(bbox[1] - bbox[3] / 2):int(bbox[1] - bbox[3] / 2) + int(bbox[3]),int(bbox[0] - bbox[2] / 2):int(bbox[0] - bbox[2] / 2) + int(bbox[2])] = mask
#     return black

def sigmoid(x):
    try:
        result = 1 / (1 + np.exp(-x))
    except:
        print(x)
        result = 0
    return result

sigmoid_v = np.vectorize(sigmoid)

outputTensor = np.ndarray((204800, 1), np.float32)

with open('/home/markc/MaskOutput.csv') as file:
    reader = csv.reader(file)
    for line in reader:
        for i in range(204800):
            outputTensor[i] = float(line[i])
            
outputTensor = outputTensor.astype(np.float64)
        
outputTensor = outputTensor.reshape((6400, 32), order='C')
# outputTensor = outputTensor.transpose((2, 0, 1))
# outputTensor = outputTensor.reshape((32, 6400))

bboxes = []
classes = []
maskWeights = []

image = cv2.imread("/home/markc/template_small.bmp").astype(np.uint8)
with open('/home/markc/Detections.csv') as file:
    reader = csv.reader(file)
    for lines in reader:
        bboxes.append([float(lines[0]) * 640, float(lines[1]) * 640, float(lines[2]) * 640, float(lines[3]) * 640])
        classes.append(int(lines[4]))
        maskWeights.append(np.array([float(num) for num in lines[5:37]]).reshape(32, 1))
        
for i in range(len(bboxes)):
    
    cv2.rectangle(image, (int(bboxes[i][0] - bboxes[i][2] / 2), int(bboxes[i][1] - bboxes[i][3] / 2)), (int(bboxes[i][0] + bboxes[i][2] / 2), int(bboxes[i][1] + bboxes[i][3] / 2)), (0, 255, 0))
    # print(maskWeights[i].T.shape)
    # print(outputTensor.shape)
    
    mask = np.matmul(outputTensor, maskWeights[i])
    mask = sigmoid_v(mask)
    # print(mask)
    mask = mask > 0.5
    mask = mask.reshape((80, 80))
    mask = mask.copy()
    # mask.resize(int(bboxes[i][3]), int(bboxes[i][2]))
    # mask = createMask(mask, bboxes[i])
    
    mask = mask * 255
    mask = mask.astype(np.uint8)
    mask = cv2.resize(mask, (640, 640), interpolation=cv2.INTER_LINEAR)
    mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    cv2.rectangle(mask, (int(bboxes[i][0] - bboxes[i][2] / 2), int(bboxes[i][1] - bboxes[i][3] / 2)), (int(bboxes[i][0] + bboxes[i][2] / 2), int(bboxes[i][1] + bboxes[i][3] / 2)), (0, 255, 0))
    
    print(classes[i])
    cv2.imshow("window", mask)
    cv2.waitKey(0)