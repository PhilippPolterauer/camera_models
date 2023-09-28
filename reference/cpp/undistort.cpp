#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <chrono>
using namespace std::chrono;

int main()
{
    // Load the precomputed maps (assuming you have them)
    cv::Mat distorted_img = cv::imread("tests/test.jpg");
    cv::FileStorage fs("tests/camera.yaml", cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        std::cerr << "Error: Could not open the file." << std::endl;
        return -1;
    }
    cv::Mat camera_matrix, distortion;
    fs["camera_matrix"] >> camera_matrix;
    fs["distortion"] >> distortion;
    fs.release();
    auto image_size = distorted_img.size();

    cv::Mat map1, map2;
    auto start = high_resolution_clock::now();
    cv::initUndistortRectifyMap(camera_matrix, distortion, cv::Mat(), camera_matrix, image_size, CV_32FC1, map1, map2);
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);
    std::cout << "init duration " << duration.count()/1e3 << "ms" << std::endl;

    // Load the distorted image

    if (distorted_img.empty())
    {
        std::cerr << "Error: Could not read the image." << std::endl;
        return -1;
    }

    // Undistort the image
    cv::Mat undistorted_img;

    start = high_resolution_clock::now();
  
    cv::remap(distorted_img, undistorted_img, map1, map2, cv::INTER_NEAREST);

    stop = high_resolution_clock::now();
    duration = duration_cast<microseconds>(stop - start);
    std::cout << "Rectification duration: " << duration.count()/1e3 << "ms" <<std::endl;

    

    cv::imwrite("undistorted_image.jpg", undistorted_img);

    return 0;
}
