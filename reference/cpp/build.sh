#! /bin/bash
gcc undistort.cpp -I/usr/include/opencv4/ -lstdc++ -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_imgcodecs -lopencv_calib3d -o undistort
