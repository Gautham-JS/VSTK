#include "config/CalibConfig.hpp"
#include "utils/Logger.hpp"

#include <opencv2/opencv.hpp>

int vstk::write_mono_config(vstk::MonoCalibConfig &cfg, std::string filename) {
    INFOLOG("Writing calibration data to file %s", filename);    
    cv::FileStorage fs = cv::FileStorage(filename, cv::FileStorage::WRITE);
    if(!fs.isOpened()) {
        return 1;
    }
    fs << "K" << cfg.K;
    fs << "DistCoeffecients" << cfg.dist_coeff;

    fs << "NCameras" << (int) cfg.Rvec.size();

    
    for(int i=0; i < cfg.Rvec.size(); i++) {
        std::stringstream ss;
        ss << i;
        fs << "R" + ss.str() << cfg.Rvec[i];
    }

    for(int i=0; i < cfg.tvec.size(); i++) {
        std::stringstream ss;
        ss << i;
        fs << "t" + ss.str() << cfg.tvec[i];
    }


    fs.release();
    return 0;
}



int vstk::write_stereo_config(vstk::CalibConfig &cfg, std::string filename) {
    INFOLOG("Writing calibration data to file %s", filename);    
    cv::FileStorage fs = cv::FileStorage(filename, cv::FileStorage::WRITE);
    if(!fs.isOpened()) {
        return 1;
    }
    fs << "Cam1K" << cfg.cam1.K;
    fs << "Cam1DistCoeffecients" << cfg.cam1.dist_coeff;

    fs << "Cam2K" << cfg.cam2.K;
    fs << "Cam2DistCoeffecients" << cfg.cam2.dist_coeff;

    fs << "EMat" << cfg.E;
    fs << "FMat" << cfg.F; 

    fs << "Rs" << cfg.Rs;
    fs << "ts" << cfg.ts;


    fs.release();
    return 0;
}