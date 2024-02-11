import cv2
import numpy as np
import time

from calibrationTools import *
from serial import Serial
from picamera2 import Picamera2

def main():
    
    imageSize = (1200, 720)
    numPoses = 41
    
    serialPort = Serial("/dev/ttyAMA0", 450)
    
    camL = Picamera2(0)
    camR = Picamera2(1)
    
    config = camL.create_video_configuration(main={"format": 'RGB888', "size": imageSize})
    camL.configure(config)
    
    config = camR.create_video_configuration(main={"format": 'RGB888', "size": imageSize})
    camR.configure(config)
    
    # camL.sensor_modes.append({"format": "RGB888"})
    # camL.sensor_modes.append({"size": imageSize})
    # camL.sensor_modes.append({"fps": 120})
    
    # camR.sensor_modes.append({"format": "RGB888"})
    # camR.sensor_modes.append({"size": imageSize})
    # camR.sensor_modes.append({"fps": 120})
    
    camL.start()
    camR.start()
    
    objp = createObjPoints(6, 8)
    
    for i in range(8):
        serialPort.write("\x00".encode())
        time.sleep(0.05)
        
    leftCornerList = []
    rightCornerList = []
    objPoints = []
        
    i = 0
    while i < numPoses:
        print("Capturing image: " + str(i))
        currentNum = 2
        keyPress = None
        # while keyPress != ord('a'):
        start = time.time()
        while currentNum > -1:
            serialPort.write("\x00".encode())
            leftIm = camL.capture_array()
            rightIm = camR.capture_array()
            
            vis = np.concatenate((leftIm, rightIm), axis=1)
            # vis = cv2.cvtColor(vis, cv2.COLOR_BGR2RGB)
            vis = cv2.putText(vis, str(currentNum), (0,60), cv2.FONT_HERSHEY_PLAIN, 4, (0, 255, 0), 5)
            vis = cv2.putText(vis, f"{i+1}/{numPoses}", (1270, 60), cv2.FONT_HERSHEY_PLAIN, 4, (0, 255, 0), 5)
            vis = cv2.resize(vis, (int(vis.shape[1] / 1.75), int(vis.shape[0] / 1.75)))
            cv2.imshow("Stereo Image", vis)
            keyPress = cv2.waitKey(10)
            if time.time() - start >= 1.0:
                currentNum -= 1
                start = time.time()
        leftGray = cv2.cvtColor(leftIm, cv2.COLOR_BGR2GRAY)
        rightGray = cv2.cvtColor(rightIm, cv2.COLOR_BGR2GRAY)
        leftFound, leftCorners = cv2.findChessboardCornersSB(leftGray, (6, 8))
        rightFound, rightCorners = cv2.findChessboardCornersSB(rightGray, (6, 8))
        if leftFound and rightFound:
            objPoints.append(objp)
            leftCornerList.append(leftCorners)
            rightCornerList.append(rightCorners)
            cv2.drawChessboardCorners(leftIm, (6, 8), leftCorners, leftFound)
            cv2.drawChessboardCorners(rightIm, (6, 8), rightCorners, rightFound)
            vis = np.concatenate((leftIm, rightIm), axis=1)
            vis = cv2.resize(vis, (int(vis.shape[1] / 1.75), int(vis.shape[0] / 1.75)))
            # vis = cv2.cvtColor(vis, cv2.COLOR_BGR2RGB)
            start = time.time()
            while time.time() - start < 2:
                cv2.imshow("Stereo Image", vis)
                cv2.waitKey(1)
            i += 1
        else:
            if leftFound:
                print("Could not find right chessboard")
            elif rightFound:
                print("Could not find left chessboard")
            else:
                print("Could not find any chessboard")
                
    leftIntrFile = cv2.FileStorage("/home/pi/Cam0Intr.xml", cv2.FILE_STORAGE_READ)
    rightIntrFile = cv2.FileStorage("/home/pi/Cam1Intr.xml", cv2.FILE_STORAGE_READ)
    camMatL = leftIntrFile.getNode("Matrix").mat()
    camDCL = leftIntrFile.getNode("DistCoeffs").mat()
    camMatR = rightIntrFile.getNode("Matrix").mat()
    camDCR = rightIntrFile.getNode("DistCoeffs").mat()
    
    flags = 0
    flags |= cv2.CALIB_FIX_INTRINSIC
    # Here we fix the intrinsic camara matrixes so that only Rot, Trns, Emat and Fmat are calculated.
    # Hence intrinsic parameters are the same 
    
    criteria_stereo= (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    
    
    # This step is performed to transformation between the two cameras and calculate Essential and Fundamenatl matrix
    retS, camMatL, camDCL, camMatR, camDCR, Rot, Trns, Emat, Fmat = cv2.stereoCalibrate(objPoints, leftCornerList, rightCornerList, camMatL, camDCL, camMatR, camDCR, imageSize, criteria_stereo, flags)
    
    if retS:
        print("Stereo calibrated")
    else:
        print("Stereo calibration failed")
        SystemExit()
    
    rect_l, rect_r, proj_mat_l, proj_mat_r, Q, roiL, roiR = cv2.stereoRectify(camMatL, camDCL, camMatR, camDCR, imageSize, Rot, Trns, 1,(0,0))
    
    Left_Stereo_Map = cv2.initUndistortRectifyMap(camMatL, camDCL, rect_l, proj_mat_l,
                                             imageSize, cv2.CV_16SC2)
    Right_Stereo_Map = cv2.initUndistortRectifyMap(camMatR, camDCR, rect_r, proj_mat_r,
                                                imageSize, cv2.CV_16SC2)
    
    print("Saving parameters ......")
    cv_file = cv2.FileStorage("/home/pi/StereoMaps.xml", cv2.FILE_STORAGE_WRITE)
    cv_file.write("Left_Stereo_Map_x",Left_Stereo_Map[0])
    cv_file.write("Left_Stereo_Map_y",Left_Stereo_Map[1])
    cv_file.write("Right_Stereo_Map_x",Right_Stereo_Map[0])
    cv_file.write("Right_Stereo_Map_y",Right_Stereo_Map[1])
    cv_file.release()
    
if __name__ == "__main__":
    main()