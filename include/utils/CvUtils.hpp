#ifndef __VSTK_CV_UTILS_H
#define __VSTK_CV_UTILS_H

#include <stdio.h>
#include <utility>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

namespace vstk {
    std::vector<cv::Mat> split_image( cv::Mat & image, int M, int N );
    std::vector<cv::Mat> split_image( cv::Mat & image, int M, int N , std::vector<std::pair<int, int>> &origins);

}

#endif