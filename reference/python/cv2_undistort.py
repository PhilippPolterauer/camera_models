import cv2
import numpy as np
import time

img = cv2.imread("tests/test.jpg")
out = np.zeros_like(img)

fs = cv2.FileStorage("tests/camera.yaml", cv2.FILE_STORAGE_READ)
camera_matrix  = fs.getNode("camera_matrix").mat()
distortion = fs.getNode("distortion").mat()
fs.release()

resolution = img.shape[1::-1]

t0 = time.time()
cv2.undistort(img, camera_matrix, distortion, out, None)
t1 = time.time()
print("cv2.undistort: ", t1 - t0)
cv2.imwrite("../results/cv2_undistort.jpg", out)

# precomputed undistortion map
map1, map2 = cv2.initUndistortRectifyMap(
    camera_matrix, distortion, None, camera_matrix, resolution, cv2.CV_32FC1
)

t0 = time.time()
cv2.remap(img, map1, map2, cv2.INTER_NEAREST, out)
t1 = time.time()
print("cv2.undistort precompute: ", t1 - t0)
cv2.imwrite("../results/cv2_undistort_precompute_nearest.jpg", out)

t0 = time.time()
cv2.remap(img, map1, map2, cv2.INTER_LINEAR, out)
t1 = time.time()
print("cv2.undistort precompute: ", t1 - t0)
cv2.imwrite("../results/cv2_undistort_precompute_linear.jpg", out)
