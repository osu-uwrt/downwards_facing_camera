import cv2

import csv

bboxes = []
classes = []

with open('/home/markc/Detections.csv') as file:
    reader = csv.reader(file)
    for lines in reader:
        bboxes.append((int(lines[0]) + 127, int(lines[1]) + 127, int(lines[2]) + 127, int(lines[3]) + 127))
        classes.append(int(lines[4]) + 127)
        
image = cv2.imread("/home/markc/yolo_im.bmp")
for i in range(len(bboxes)):
    print((bboxes[i][0] - bboxes[i][2] / 2, bboxes[i][1] - bboxes[i][3] / 2))
    cv2.rectangle(image, (int(bboxes[i][0] - bboxes[i][2] / 2), int(bboxes[i][1] - bboxes[i][3] / 2)), (int(bboxes[i][0] + bboxes[i][2] / 2), int(bboxes[i][1] + bboxes[i][3] / 2)), (0, 255, 0))
    
cv2.imshow("Image", image)
cv2.waitKey(0)