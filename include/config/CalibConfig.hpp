#include <opencv2/core.hpp>
#include <vector>

#include "config/Config.hpp"

#ifndef __VSTK_CALIB3D_CONF
#define __VSTK_CALIB3D_CONF

namespace vstk {

    enum CALIB_MODE {
        MONO_CAM,
        STEREO_CAM
    };

    typedef struct MonoCalibConfig {        
        std::string dir_pattern;
        std::vector<std::string> filenames;
    } MonoCalibConfig;

    typedef struct CalibConfig {
        MonoCalibConfig cam1, cam2;
        int nrows, ncols;
        std::string out_dir;
        std::string out_fname = "stereo_params.yaml";
        CALIB_MODE mode;
    } CalibConfig;

    int write_mono_params(MonoCamParams &params, std::string filepath);
    int write_stereo_params(StereoCamParams &params, std::string filepath);
    
}

#endif
