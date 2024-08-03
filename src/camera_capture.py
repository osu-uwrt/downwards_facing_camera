import os
import re
import time
import cv2
from picamera2 import Picamera2
import serial

def create_camera(camera_id):
    picam2 = Picamera2(camera_id)
    config = picam2.create_video_configuration(main={"size": (640, 360)})
    picam2.configure(config)
    picam2.start()
    return picam2

def get_starting_counter(directory, pattern_str):
    files = os.listdir(directory)
    counter = 1
    pattern = re.compile(pattern_str)

    counters = [int(pattern.match(f).group(1)) for f in files if pattern.match(f)]
    if counters:
        counter = max(counters) + 1

    return counter

def main():
    
    ser = serial.Serial("/dev/ttyAMA4", 600)
    
    try:
        camL = create_camera(0)
        camR = create_camera(1)
    except Exception as e:
        print(f"Error initializing cameras: {e}")
        return
    
    time.sleep(1)
    for i in range(16):
        ser.write(b'\x00')

    # Define parent directory
    parent_dir = os.path.join(os.path.dirname(os.getcwd()), 'captures')
    # Ensure directories exist
    os.makedirs(os.path.join(parent_dir, 'imgs/Left'), exist_ok=True)
    os.makedirs(os.path.join(parent_dir, 'imgs/Right'), exist_ok=True)
    os.makedirs(os.path.join(parent_dir, 'videos/Left'), exist_ok=True)
    os.makedirs(os.path.join(parent_dir, 'videos/Right'), exist_ok=True)

    image_counter = get_starting_counter(os.path.join(parent_dir, 'imgs/Left'), r'left_(\d+)\.png')
    video_counter = get_starting_counter(os.path.join(parent_dir, 'videos/Left'), r'left_(\d+)\.avi')

    recording = False
    video_writerL = None
    video_writerR = None
    
    key = ord('v')
    
    start = time.time()

    try:
        while True:
            ser.write(b'\x00')
            frameL = camL.capture_array()
            frameR = camR.capture_array()

            # cv2.imshow('Left Camera', frameL)
            # cv2.imshow('Right Camera', frameR)

            # key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == 13:  # Enter key
                left_filename = os.path.join(parent_dir, f'imgs/Left/left_{image_counter}.png')
                right_filename = os.path.join(parent_dir, f'imgs/Right/right_{image_counter}.png')
                cv2.imwrite(left_filename, frameL)
                cv2.imwrite(right_filename, frameR)
                print(f"Saved {left_filename} and {right_filename}")
                image_counter += 1
            elif key == ord('v'):
                if not recording:
                    recording = True
                    video_writerL = cv2.VideoWriter(os.path.join(parent_dir, f'videos/Left/left_{video_counter}.avi'), cv2.VideoWriter_fourcc(*'MJPG'), 30, (640, 360))
                    video_writerR = cv2.VideoWriter(os.path.join(parent_dir, f'videos/Right/right_{video_counter}.avi'), cv2.VideoWriter_fourcc(*'MJPG'), 30, (640, 360))
                    if not video_writerL.isOpened() or not video_writerR.isOpened():
                        print("Error: VideoWriter did not open")
                        recording = False
                        continue
                    print(f"Started recording video {video_counter}")
                    key = 0
                else:
                    recording = False
                    video_writerL.release()
                    video_writerR.release()
                    print(f"Stopped recording video {video_counter}")
                    video_counter += 1
                    break

            if recording:
                if frameL is not None and frameR is not None:
                    print(f"FrameL size: {frameL.shape}, FrameR size: {frameR.shape}")
                    # Convert frames to 3 channels (BGR)
                    frameL_bgr = cv2.cvtColor(frameL, cv2.COLOR_RGBA2BGR)
                    frameR_bgr = cv2.cvtColor(frameR, cv2.COLOR_RGBA2BGR)
                    video_writerL.write(frameL_bgr)
                    video_writerR.write(frameR_bgr)
                else:
                    print("Error: Frame is None, skipping write")
            if (time.time() - start >= 5):
                key = ord('v')

    except Exception as e:
        print(f"Error during capture: {e}")
    finally:
        if recording:
            video_writerL.release()
            video_writerR.release()
            left_video_filename = os.path.join(parent_dir, f'videos/Left/left_{video_counter}.avi')
            right_video_filename = os.path.join(parent_dir, f'videos/Right/right_{video_counter}.avi')

        camL.stop()
        camR.stop()
        cv2.destroyAllWindows()
        print("Cameras stopped")

if __name__ == "__main__":
    main()
