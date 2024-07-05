#ifndef __VSTK_DATA_UTILS_H_
#define __VSTK_DATA_UTILS_H_

#include <string>
#include <vector>
#include <opencv2/core.hpp>

#include "utils/Logger.hpp"

namespace vstk {
    typedef struct FPSDataPt_t {
        std::string frame_id = nullptr;
        int idx = 0;
        int fps = 0;
        bool is_insertion = false;
    } FPSDataPt_t;


    FPSDataPt_t create_fps_data_pt(std::string frame_id, int fps, bool is_insertion);
    
    inline FPSDataPt_t create_fps_data_pt(std::string frame_id, int fps) {
        return create_fps_data_pt(frame_id, fps, false);
    }

    void dump_data_pts_to_file(std::string filename, std::vector<vstk::FPSDataPt_t> data_pts);
}

#endif