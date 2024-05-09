#include "config/CalibConfig.hpp"
#include "utils/Logger.hpp"

#include <opencv2/opencv.hpp>

int vstk::write_mono_params(vstk::MonoCamParams &params, std::string filename) {
    INFOLOG("Writing calibration data to file %s", filename);    
    cv::FileStorage fs = cv::FileStorage(filename, cv::FileStorage::WRITE);
    if(!fs.isOpened()) {
        return 1;
    }
    fs << "K" << params.K;
    fs << "DistCoeffecients" << params.dist_coeff;

    fs << "NCameras" << (int) params.R_vecs.size();

    
    for(int i=0; i < params.R_vecs.size(); i++) {
        std::stringstream ss;
        ss << i;
        fs << "R" + ss.str() << params.R_vecs[i];
    }

    for(int i=0; i < params.t_vecs.size(); i++) {
        std::stringstream ss;
        ss << i;
        fs << "t" + ss.str() << params.t_vecs[i];
    }
    fs.release();
    return 0;
}



int vstk::write_stereo_params(vstk::StereoCamParams &params, std::string filename) {
    INFOLOG("Writing calibration data to file %s", filename);    
    cv::FileStorage fs = cv::FileStorage(filename, cv::FileStorage::WRITE);
    if(!fs.isOpened()) {
        return 1;
    }
    fs << "Cam1K" << params.cam1_params.K;
    fs << "Cam1DistCoeffecients" << params.cam1_params.dist_coeff;

    fs << "Cam2K" << params.cam2_params.K;
    fs << "Cam2DistCoeffecients" << params.cam2_params.dist_coeff;

    fs << "EMat" << params.E;
    fs << "FMat" << params.F; 

    fs << "Rs" << params.Rs;
    fs << "ts" << params.ts;


    fs.release();
    return 0;
}

int read_stereo_config(vstk::CalibConfig &cfg, std::string filename) {
    INFOLOG("Reading camera parameters");
    return 0;
}