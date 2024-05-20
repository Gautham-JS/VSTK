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
    
    fs.startWriteStruct("cam_params", cv::FileNode::MAP);
    fs.startWriteStruct("stereo_cam_params", cv::FileNode::MAP);
    fs.startWriteStruct("left_cam", cv::FileNode::MAP);     

    fs << "K" << params.cam1_params.K;
    fs << "dist_coeff" << params.cam1_params.dist_coeff;
    
    fs.endWriteStruct();                                                // close left cam intrinsics
    fs.startWriteStruct("right_cam", cv::FileNode::MAP);                // start right cam intrinsics

    fs << "K" << params.cam2_params.K;
    fs << "dist_coeff" << params.cam2_params.dist_coeff;
    
    fs.endWriteStruct();                                                // close right cam intrinsics
    
    // Extrinsic params shared between cameras.

    fs << "EMat" << params.E;                                           // Essential Matrix
    fs << "FMat" << params.F;                                           // Fundamental Matrix

    fs << "Rs" << params.Rs;                                            // Cam Rotation Matrix (3x3)
    fs << "ts" << params.ts;                                            // Cam Translation Vector (3x1)

    fs.endWriteStruct();                                                // close "stereo_cam_params" section
    fs.endWriteStruct();                                                // close "cam_params" section

    fs.release();
    return 0;
}

int read_stereo_config(vstk::CalibConfig &cfg, std::string filename) {
    INFOLOG("Reading camera parameters");
    return 0;
}
