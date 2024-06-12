import cv2
import numpy as np

def createObjPoints(width: int, height: int):
    objp = np.zeros((width * height, 3), np.float32)
    objp[:, :2] = np.mgrid[0:width, 0:height].T.reshape(-1, 2)
    
    return objp

leftCornersList = []
rightCornersList = []
objPoints = []

images_folder = "./Air_Cal_Imgs/Good_StereoAir/"

for i in range(36):
    leftim = cv2.imread(images_folder + f"Left_{i+1}.png")
    rightim = cv2.imread(images_folder + f"Right_{i+1}.png")
    
    leftFound, leftCorners = cv2.findChessboardCornersSB(leftim, (6, 8))
    rightFound, rightCorners = cv2.findChessboardCornersSB(rightim, (6, 8))
    
    if leftFound and rightFound:
        objPoints.append(createObjPoints(6, 8))
        leftCornersList.append(leftCorners)
        rightCornersList.append(rightCorners)
        
        cv2.drawChessboardCorners(leftim, (6, 8), leftCorners, leftFound)
        cv2.drawChessboardCorners(rightim, (6, 8), rightCorners, rightFound)
        
        vis = np.concatenate((leftim, rightim), axis=1)
        
        cv2.imshow("Images", vis)
        cv2.waitKey(0)
        
leftIntrFile = cv2.FileStorage("./Cam0Intr.xml", cv2.FILE_STORAGE_READ)
rightIntrFile = cv2.FileStorage("./Cam1Intr.xml", cv2.FILE_STORAGE_READ)
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
retS, camMatL, camDCL, camMatR, camDCR, Rot, Trns, Emat, Fmat = cv2.stereoCalibrate(objPoints, leftCornersList, rightCornersList, camMatL, camDCL, camMatR, camDCR, (640, 360), criteria_stereo, flags)

if retS:
    print("Stereo calibrated")
else:
    print("Stereo calibration failed")
    SystemExit()

rect_l, rect_r, proj_mat_l, proj_mat_r, Q, roiL, roiR = cv2.stereoRectify(camMatL, camDCL, camMatR, camDCR, (640, 360), Rot, Trns, 1,(0,0))

print(Trns)

Left_Stereo_Map = cv2.initUndistortRectifyMap(camMatL, camDCL, rect_l, proj_mat_l,
                                            (640, 360), cv2.CV_16SC2)
Right_Stereo_Map = cv2.initUndistortRectifyMap(camMatR, camDCR, rect_r, proj_mat_r,
                                            (640, 360), cv2.CV_16SC2)

print("Saving parameters ......")
cv_file = cv2.FileStorage("./StereoMaps.xml", cv2.FILE_STORAGE_WRITE)
cv_file.write("Left_Stereo_Map_x",Left_Stereo_Map[0])
cv_file.write("Left_Stereo_Map_y",Left_Stereo_Map[1])
cv_file.write("Right_Stereo_Map_x",Right_Stereo_Map[0])
cv_file.write("Right_Stereo_Map_y",Right_Stereo_Map[1])
cv_file.write("Q", Q)
cv_file.release()
    