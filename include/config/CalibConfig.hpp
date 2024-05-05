#include <opencv2/core.hpp>
#include <vector>

#ifndef __VSTK_CALIB3D_CONF
#define __VSTK_CALIB3D_CONF

namespace vstk {

    enum CALIB_MODE {
        MONO,
        STEREO
    };

    typedef struct MonoCameraParams {
        cv::Mat K, dist_coeff;
        std::vector<cv::Mat> R_vecs, t_vecs;
        int n_rows, n_cols;
    } MonoCamParams;

    typedef struct StereoCameraParams {
        MonoCamParams cam1_params, cam2_params;
        cv::Mat Rs, ts, E, F;
        int n_rows, n_cols;
    } StereoCamParams;

    typedef struct MonoCalibConfig {        
        std::string dir_pattern;
        std::vector<std::string> filenames;
    } MonoCalibConfig;

    typedef struct CalibConfig {
        MonoCalibConfig cam1, cam2;
        int nrows, ncols;
        std::string out_dir;
        CALIB_MODE mode;
    } CalibConfig;

    int write_mono_params(MonoCamParams &params, std::string filepath);
    int write_stereo_params(StereoCamParams &params, std::string filepath);
    
}

#endif