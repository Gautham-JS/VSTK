#ifndef __VSTK_CV_UTILS_H
#define __VSTK_CV_UTILS_H

#include <stdio.h>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

namespace vstk {
    std::vector<cv::Mat> split_image( cv::Mat & image, int M, int N );
    std::vector<cv::Mat> split_image( cv::Mat & image, int M, int N , std::vector<std::pair<int, int>> &origins);
    void join_image_in_another(cv::Mat &parent, cv::Mat child, std::pair<int, int> child_origin_in_parent);
    cv::Mat undistort_image(cv::Mat image, cv::Mat camera_matrix);
    int normalize(int value, int min_range, int max_range);
    float normalize_float(float value, float min_range, float max_range);
    void write_ply(std::vector<cv::Point3f> points, std::string filename);
    void draw_depth(cv::Mat image, std::vector<cv::Point2f> p2d, std::vector<cv::Point3f> p3d);
    void draw_reprojection(cv::Mat image, cv::Mat P, std::vector<cv::Point2f> p2d, cv::Mat p4d);

    void clip_pcl(std::vector<cv::Point3f> pts, std::vector<cv::Point3f> &out_pts, int min_clip_depth, int max_clip_depth);
}

#endif