import cv2

import os

import numpy as np

cornersList = []
objList = []

def createObjPoints(width: int, height: int):
    objp = np.zeros((width * height, 3), np.float32)
    objp[:, :2] = np.mgrid[0:width, 0:height].T.reshape(-1, 2)
    
    return objp

imageFolder = "./Air_Cal_Imgs/Good_Cam0_Air/"

for image in os.listdir(imageFolder):
    if (image.endswith(".png")):
        cvImage = cv2.imread(imageFolder + image)
        cvImage = cv2.cvtColor(cvImage, cv2.COLOR_RGB2BGR)
        found, corners = cv2.findChessboardCornersSB(cvImage, (6, 8))
        if found:
            # cv2.drawChessboardCorners(cvImage, (6, 8), corners, found)
            # cv2.imshow("window", cvImage)
            # cv2.waitKey(0)
            cornersList.append(corners)
            objList.append(createObjPoints(6, 8))
        else:
            print("Not found in " + image)
            

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objList, cornersList, (640, 360), None, None)

# Use this if you're feeling extra fancy (It gives a weird result hense why there's an roi)
newMat, roi = cv2.getOptimalNewCameraMatrix(
            mtx, dist, (640, 360), 1, (640, 360)
        )
# print(mtx)
# print(newMat)

# Change this depending on camera
cvFile = cv2.FileStorage(f"./Cam0Intr.xml", cv2.FileStorage_WRITE)
cvFile.write("Matrix", mtx)
cvFile.write("DistCoeffs", dist)
cvFile.release()