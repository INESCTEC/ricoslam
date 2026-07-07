#pragma once

// OpenCV
#include <opencv2/opencv.hpp>

namespace inesctec_mrdt_slam_distmap_2d
{

namespace data_structures
{

//! OpenCV keys definitions
#define OPENCV_KEY_UP 2490368
#define OPENCV_KEY_DOWN 2621440
#define OPENCV_KEY_LEFT 2424832
#define OPENCV_KEY_RIGHT 2555904
#define OPENCV_KEY_SPACE 32
#define OPENCV_KEY_DELETE 3014656
#define OPENCV_KEY_ESCAPE 27

//! OpenCV image definitions
using CvMatImage = cv::Mat;
using UCharImage = cv::Mat_<unsigned char>;
using IntImage = cv::Mat_<int>;
using FloatImage = cv::Mat_<float>;
using RGBImage = cv::Mat_<cv::Vec3b>;

}  // namespace data_structures

}  // namespace inesctec_mrdt_slam_distmap_2d
