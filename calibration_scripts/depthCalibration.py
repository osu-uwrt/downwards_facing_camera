import cv2
import matplotlib.pyplot as plt
import numpy as np

z = np.array([1.3716, 1.4859, 1.6002, 1.7145, 1.8288, 1.9431, 2.0574, 2.1717, 2.286]).astype(np.float32)
coeff = np.array([111, 99, 95, 86, 80, 75, 71, 68, 62]).astype(np.float32)
coeff = 1 / coeff

# plt.plot(z, coeff)
# plt.show()

ret, sol = cv2.solve(coeff,z,flags=cv2.DECOMP_QR)

print(sol)