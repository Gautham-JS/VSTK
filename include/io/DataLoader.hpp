#ifndef __VSTK_DATA_LOADER_H
#define __VSTK_DATA_LOADER_H

#include <stdio.h>
#include <string>

#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"

namespace vstk {
    class DataLoader {
        public:
            virtual int load_source(std::string source);
            virtual int iterate_next_image(cv::Mat &image);
            virtual bool is_iteration_complete();
    };
}

#endif