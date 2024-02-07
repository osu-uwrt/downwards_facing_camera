import cv2
import numpy as np

from calibrationTools import *
from serial import Serial
from picamera2 import Picamera2

def main():
    
    serialPort = Serial("/dev/ttyAMA0", 450)
    
    camL = Picamera2(0)
    camR = Picamera2(1)
    
    camL.sensor_modes.append({"format": "RGB888"})
    camL.sensor_modes.append({"size": (1456, 1088)})
    camL.sensor_modes.append({"fps": 120})
    
    camR.sensor_modes.append({"format": "RGB888"})
    camR.sensor_modes.append({"size": (1456, 1088)})
    camR.sensor_modes.append({"fps": 120})
    
    camL.start()
    camR.start()
    
    objp = createObjPoints(6, 8)
    
    for i in range(8):
        serialPort.writelines("\x00".encode())
        serialPort.flush()
        
    leftCornerList = []
    rightCornerList = []
    objPoints = []
        
    i = 0
    while i < 24:
        print("Capturing image: " + str(i))
        keyPress = None
        while keyPress != ord('a'):
            serialPort.writelines("\x00".encode())
            leftIm = camL.capture_array()
            rightIm = camR.capture_array()
            
            vis = np.concatenate((leftIm, rightIm), axis=1)
            vis = cv2.cvtColor(vis, cv2.COLOR_BGR2RGB)
            cv2.imshow("Stereo Image", vis)
            keyPress = cv2.waitKey(10)
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
            vis = cv2.cvtColor(vis, cv2.COLOR_BGR2RGB)
            cv2.imshow("Stereo Image", vis)
            cv2.waitKey(0)
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
    retS, camMatL, camDCL, camMatR, camDCR, Rot, Trns, Emat, Fmat = cv2.stereoCalibrate(objPoints, leftCornerList, rightCornerList, camMatL, camDCL, camMatR, camDCR, (1456, 1088), criteria_stereo, flags)
    
    if retS:
        print("Stereo calibrated")
    else:
        print("Stereo calibration failed")
        SystemExit()
    
    rect_l, rect_r, proj_mat_l, proj_mat_r, Q, roiL, roiR = cv2.stereoRectify(camMatL, camDCL, camMatR, camDCR, (1456, 1088), Rot, Trns, 1,(0,0))
    
    Left_Stereo_Map = cv2.initUndistortRectifyMap(camMatL, camDCL, rect_l, proj_mat_l,
                                             (1456, 1088), cv2.CV_16SC2)
    Right_Stereo_Map = cv2.initUndistortRectifyMap(camMatR, camDCR, rect_r, proj_mat_r,
                                                (1456, 1088), cv2.CV_16SC2)
    
    print("Saving parameters ......")
    cv_file = cv2.FileStorage("/home/pi/StereoMaps.xml", cv2.FILE_STORAGE_WRITE)
    cv_file.write("Left_Stereo_Map_x",Left_Stereo_Map[0])
    cv_file.write("Left_Stereo_Map_y",Left_Stereo_Map[1])
    cv_file.write("Right_Stereo_Map_x",Right_Stereo_Map[0])
    cv_file.write("Right_Stereo_Map_y",Right_Stereo_Map[1])
    cv_file.release()
    
if __name__ == "__main__":
    main()