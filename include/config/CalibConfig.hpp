#include <opencv2/core.hpp>
#include <vector>

#ifndef __VSTK_CALIB3D_CONF
#define __VSTK_CALIB3D_CONF

namespace vstk {

    enum CALIB_MODE {
        MONO,
        STEREO
    };

    typedef struct MonoCalibConfig {
        cv::Mat K, dist_coeff;
        std::vector<cv::Mat> Rvec, tvec;        
        std::string dir_pattern;
        std::vector<std::string> filenames;
    } MonoCalibConfig;

    typedef struct CalibConfig {
        MonoCalibConfig cam1, cam2;
        cv::Mat Rs, ts, E, F;
        std::string out_dir;
        int ncols;
        int nrows;
        CALIB_MODE mode;
    } CalibConfig;

    int write_mono_config(MonoCalibConfig &cfg, std::string filepath);
    int write_stereo_config(CalibConfig &cfg, std::string filepath);
    
}

#endif