import os
import pathlib

from pycoral.utils import edgetpu
from pycoral.utils import dataset
from pycoral.adapters import common

from PIL import Image

import numpy as np

model_file = "/home/pi/yolo_model/yolov8n-seg_full_integer_quant_edgetpu.tflite"

interpreter = edgetpu.make_interpreter(model_file)
interpreter.allocate_tensors()

image = Image.open("/home/pi/yolo_im.bmp").convert('RGB')

image = np.array(image)

print(image.shape)