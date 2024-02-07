import cv2
import sys
import numpy as np

from picamera2 import Picamera2
from serial import Serial
from calibrationTools import *

def main():

    serialPort = Serial("/dev/ttyAMA0", 450)

    camera = None
    id = 0

    if len(sys.argv) == 2:
        if int(sys.argv[1]) == 0 or int(sys.argv[1]) == 1:
            camera = Picamera2(int(sys.argv[1]))
            print("Calibrating camera " + sys.argv[1])
            id = int(sys.argv[1])
        else:
            print(int(sys.argv[1]) == 1)
            print("Invalid arguement: " + sys.argv[1])
            SystemExit()
    else:
        camera = Picamera2()
        print("No camera specified, defaulting to 0")
    # config = camera.create_video_configuration(main={"format": 'RGB888', "size": (1456, 1088)})
    camera.sensor_modes.append({"format": "RGB888"})
    camera.sensor_modes.append({"size": (1456, 1088)})
    camera.sensor_modes.append({"fps": 120})

    # camera.configure(config)

    camera.start()
    
    for i in range(8):
        serialPort.write(b'\x00')
        serialPort.flush()

    keyPress = ord("0")
    found = False
    image = []
    cornerList = []
    objPoints = []
    i = 0

    objp = createObjPoints(6, 8)

    while i < 39:

        print("Current Image: " + str(i + 1))

        while keyPress != ord("a"):
            serialPort.write(b'\x00')
            serialPort.flush()
            image = camera.capture_array()
            displayIm = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            cv2.imshow("Feed", displayIm)
            keyPress = cv2.waitKey(10)
        image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        found, corners = cv2.findChessboardCornersSB(image, (6, 8))
        if found:
            cv2.drawChessboardCorners(displayIm, (8, 6), corners, found)
            cv2.imshow("Feed", displayIm)
            cv2.waitKey(0)
            objPoints.append(objp)
            cornerList.append(corners)
            i += 1
        else:
            print("Could not find chessboard")
        keyPress = None

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objPoints, cornerList, image.shape[::-1], None, None
    )
    if ret:
        cv2.destroyAllWindows()
        print("Success!")
        newMat, roi = cv2.getOptimalNewCameraMatrix(
            mtx, dist, (1456, 1088), 1, (1456, 1088)
        )
        cvFile = cv2.FileStorage(f"/home/pi/Cam{id}Intr.xml", cv2.FileStorage_WRITE)
        cvFile.write("Matrix", newMat)
        cvFile.write("DistCoeffs", dist)
        cvFile.release()


if __name__ == "__main__":
    main()
