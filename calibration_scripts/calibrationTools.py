import cv2
import numpy as np


"""
Creates object points for a chessboard
"""
def createObjPoints(width: int, height: int):
    objp = np.zeros((width * height, 3), np.float32)
    objp[:, :2] = np.mgrid[0:width, 0:height].T.reshape(-1, 2)
    
    return objp

