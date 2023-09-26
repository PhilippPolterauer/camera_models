import cv2
import numpy as np
import time

img = cv2.imread("tests/test.jpg")
out = np.zeros_like(img)

camera_matrix = np.array(
    [
        [1244.617161547647, 0.0, 2016.0],
        [0.0, 930.993392665601, 1508.0],
        [0.0, 0.0, 1.0],
    ]
)
distortion = np.array([0.1, 0.1, 0.2, 0.1, 0.0])


t0 = time.time()
cv2.undistort(img, camera_matrix, distortion, out, None)
t1 = time.time()
print("cv2.undistort: ", t1 - t0)
cv2.imwrite("results/cv2_undistort.jpg", out)

# precomputed undistortion map
map1, map2 = cv2.initUndistortRectifyMap(
    camera_matrix, distortion, None, camera_matrix, (4032, 3024), cv2.CV_32FC1
)

t0 = time.time()
cv2.remap(img, map1, map2, cv2.INTER_NEAREST, out)
t1 = time.time()
print("cv2.undistort precompute: ", t1 - t0)
cv2.imwrite("results/cv2_undistort_precompute_nearest.jpg", out)

t0 = time.time()
cv2.remap(img, map1, map2, cv2.INTER_LINEAR, out)
t1 = time.time()
print("cv2.undistort precompute: ", t1 - t0)
cv2.imwrite("results/cv2_undistort_precompute_linear.jpg", out)

