import numpy as np
import cv2
import glob
from pydantic import BaseModel
import toml
import json


class Pinhole(BaseModel):
    fx: float
    fy: float
    cx: float
    cy: float
    s: float

class PlumbBob(BaseModel):
    k1: float
    k2: float
    p1: float
    p2: float
    k3: float

class FishEye(BaseModel):
    k1: float
    k2: float
    k3: float
    k4: float


def calibrate_dataset(path, num_corners_x, num_corners_y, wait_time=100):
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    calib_flags = (
        cv2.CALIB_CB_ADAPTIVE_THRESH
        + cv2.CALIB_CB_NORMALIZE_IMAGE
        + cv2.CALIB_CB_FAST_CHECK
    )
    # Define the number of corners in the calibration pattern

    # Create arrays to store object points and image points from all images
    obj_points = []  # 3D points in real world space
    img_points = []  # 2D points in image plane

    # Generate object points (3D points in real world space)
    objp = np.zeros((num_corners_x * num_corners_y, 3), np.float32)
    objp[:, :2] = np.mgrid[0:num_corners_x, 0:num_corners_y].T.reshape(-1, 2)

    # Get a list of calibration images
    calibration_images = glob.glob(path)  # List of calibration images
    print(calibration_images)
    cv2.namedWindow("Calibration Image")
    # Load and process each calibration image
    for i in range(len(calibration_images)):
        img = cv2.imread(calibration_images[i])
        cv2.imshow("Calibration Image", img)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        cv2.waitKey(1)
        # Find chessboard corners
        ret, corners = cv2.findChessboardCornersSB(
            gray, (num_corners_x, num_corners_y), cv2.CALIB_CB_EXHAUSTIVE
        )

        if ret == True:
            refined_corners = cv2.cornerSubPix(
                gray, corners, (5, 5), (-1, -1), criteria
            )
            obj_points.append(objp)
            img_points.append(corners)

            # Draw and display the corners
            cv2.drawChessboardCorners(img, (num_corners_x, num_corners_y), corners, ret)
            cv2.imshow("Chessboard Corners", img)
            cv2.waitKey(wait_time)

    cv2.destroyAllWindows()

    # Calibrate the camera
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, gray.shape[::-1], None, None
    )

    print("Calibration matrix:")
    print(mtx)
    print("Distortion coefficients:")
    print(dist)
    print("reprj. error:")
    print(ret)

    # Reprojection Error
    distances = []
    for opts, rvec, tvec, img_pts in zip(obj_points, rvecs, tvecs, img_points):
        img_pts
        reproject_pts, _ = cv2.projectPoints(opts, rvec, tvec, mtx, dist)

        distance = np.linalg.norm((img_pts - reproject_pts).reshape((-1, 2)), axis=1)
        print(distance.mean())
        distances.append(distance)

    print("Mean reprojection error: ", np.mean(distances))
    print("RMS reprojection error: ", np.sqrt(np.mean(np.square(distances))))

    return mtx, dist


# calibrate_dataset("datasets/matlab/calib_doc/htmls/calib_example/*.tif", 13, 12)


mtx, dist = calibrate_dataset("datasets/smatt/Left_bmp/*.bmp", 12, 12, 1)
projection = Pinhole(fx=mtx[0, 0], fy=mtx[1, 1], cx=mtx[0, 2], cy=mtx[1, 2], s=mtx[0, 1])
distortion = PlumbBob(k1=dist[0], k2=dist[1], p1=dist[2], p2=dist[3], k3=dist[4])
with open("datasets/smatt/calib.toml", "w") as file:
    toml.dump(
        {
            "projection": projection.model_dump(),
            "distortion": {"type": "PlumbBob", "coefficients": PlumbBob.model_dump(distortion)},
        },
        file,
    )
