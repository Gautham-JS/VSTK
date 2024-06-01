#ifndef __VSTK_CV_UTILS_H
#define __VSTK_CV_UTILS_H

#include <stdio.h>
#include <utility>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

namespace vstk {
    std::vector<cv::Mat> split_image( cv::Mat & image, int M, int N );
    std::vector<cv::Mat> split_image( cv::Mat & image, int M, int N , std::vector<std::pair<int, int>> &origins);
    void join_image_in_another(cv::Mat &parent, cv::Mat child, std::pair<int, int> child_origin_in_parent);
    cv::Mat undistort_image(cv::Mat image, cv::Mat camera_matrix);
    int normalize(int value, int min_range, int max_range);
    void write_ply(std::vector<cv::Point3f> points, std::string filename);

}

#endif