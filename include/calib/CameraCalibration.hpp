#include "config/CalibConfig.hpp"

#include <stdlib.h>
#include <vector>
#include <utility>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

#ifndef __VSTK_CALIB3D_H
#define __VSTK_CALIB3D_H

namespace vstk {
    
    class CameraCalibrator {
        private:
            std::pair<int, int> checkerboard_size;
            void calc_obj_and_img_pts(
                MonoCalibConfig &cfg,
                std::vector<cv::Point2f> &corner_pts,
                std::vector<std::vector<cv::Point3f>> &obj_pts,
                std::vector<std::vector<cv::Point2f>> &image_pts
            );
        public:
            CameraCalibrator(std::pair<int, int> cb_dimensions); 
            void run(MonoCalibConfig &cfg);
            void run(CalibConfig &cfg);
    };

}

#endif