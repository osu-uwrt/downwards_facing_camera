import cv2
import sys
import time

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
    
    # camera.sensor_modes.append({"size": (640, 640), "fps": 1})
    config = camera.create_video_configuration(main={"format": 'RGB888', "size": (1200, 720)})
    camera.configure(config)
    # print(camera.sensor_modes)
    # camera.sensor_modes.append({"format": "RGB888"})
    # camera.sensor_modes.append({"fps": 60})
    # camera.sensor_modes.append({"size": (1088, 1456)})
    
    # controls = Controls(camera)
    # controls
    # print(len(camera.sensor_modes))

    camera.start()
    
    for i in range(8):
        serialPort.write('\x00'.encode())
        time.sleep(.05)

    keyPress = ord("0")
    found = False
    image = []
    cornerList = []
    objPoints = []
    i = 0

    objp = createObjPoints(6, 8)
    cv2.namedWindow("Feed", cv2.WINDOW_NORMAL)
    # cv2.resizeWindow("Feed", 920, 920)
    

    while i < 39:

        print("Current Image: " + str(i + 1))
        currentNum = 2

        # while keyPress != ord("a"):
        start = time.time()
        while currentNum > -1:
            serialPort.write('\x00'.encode())
            image = camera.capture_array()
            # displayIm = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            displayIm = cv2.putText(image, str(currentNum), (0,60), cv2.FONT_HERSHEY_PLAIN, 4, (0, 255, 0), 5)
            displayIm = cv2.putText(displayIm, f"{i+1}/39", (1270, 60), cv2.FONT_HERSHEY_PLAIN, 4, (0, 255, 0), 5)
            # print(displayIm.shape)
            cv2.imshow("Feed", displayIm)
            keyPress = cv2.waitKey(10)
            if time.time() - start >= 1.0:
                currentNum -= 1
                start = time.time()
        image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        found, corners = cv2.findChessboardCornersSB(image, (6, 8))
        if found:
            cv2.drawChessboardCorners(displayIm, (8, 6), corners, found)
            start = time.time()
            while time.time() - start < 2:
                cv2.imshow("Feed", displayIm)
                cv2.waitKey(1)
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
