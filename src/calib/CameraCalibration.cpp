#include "calib/CameraCalibration.hpp"
#include "utils/Logger.hpp"

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <unordered_set>

using namespace vstk;



CameraCalibrator::CameraCalibrator(std::pair<int, int> cb_dimensions) : checkerboard_size(cb_dimensions) {}


void calc_obj_and_img_pts(
                MonoCalibConfig &cfg,
                std::vector<cv::Point2f> &corner_pts,
                std::vector<std::vector<cv::Point3f>> &obj_pts,
                std::vector<std::vector<cv::Point2f>> &image_pts ) {

}

MonoCamParams CameraCalibrator::run(MonoCalibConfig &cfg) {
    cv::Mat frame, gray;
    std::vector<cv::Point2f> corner_pts;
    std::vector<std::vector<cv::Point3f>> obj_pts;
    std::vector<std::vector<cv::Point2f>> image_pts;
    MonoCamParams params;
    bool success;
    

    cv::glob(cfg.dir_pattern, cfg.filenames);

    std::vector<cv::Point3f> objp;
    for(int i{0}; i<checkerboard_size.second; i++) {
        for(int j{0}; j<checkerboard_size.first; j++)
        objp.push_back(cv::Point3f(j,i,0));
    }

    INFOLOG("Running calibration for single camera with %ld frames and checkerboard dimensions [%d X %d]", cfg.filenames.size(), checkerboard_size.first, checkerboard_size.second);

    for (int i=0; i<cfg.filenames.size(); i+=40) {
        frame = cv::imread(cfg.filenames[i]);
        INFOLOG("Processing image %s", cfg.filenames[i]);
        cv::cvtColor(frame,gray,cv::COLOR_BGR2GRAY);
        success = cv::findChessboardCorners(gray, cv::Size(checkerboard_size.first, checkerboard_size.second), corner_pts, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);
        if(success) {
            cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001);
            cv::cornerSubPix(gray, corner_pts, cv::Size(11,11), cv::Size(-1,-1),criteria);
            cv::drawChessboardCorners(frame, cv::Size(checkerboard_size.first, checkerboard_size.second), corner_pts, success);
            obj_pts.push_back(objp);
            image_pts.push_back(corner_pts);
        }
        else {
            WARNLOG("No checkerboard detected in view");
        }
        cv::imshow("Image",frame);
        int key = (cv::waitKey(1) & 0xFF);
        if(key == 'q') {
            INFOLOG("Caught 'q' keypress, exitting process.");
            cv::destroyAllWindows();
            exit(0);
        }
    }
    INFOLOG("Iterative point detection complete, computing intrinsic and extrinsic camera matrices...");

    cv::destroyAllWindows();

    INFOLOG("Object Points : %ld, Image Points : %ld, starting calibration compute...", obj_pts.size(), image_pts.size());
    cv::calibrateCamera(obj_pts, image_pts, cv::Size(gray.rows, gray.cols), params.K, params.dist_coeff, params.R_vecs, params.t_vecs);
    INFOLOG("Done!");

    std::cout << "cameraMatrix : " << params.K << std::endl;
    std::cout << "distCoeffs : " << params.dist_coeff << std::endl;
    std::cout << "Rotation vector size : " << params.R_vecs.size() << std::endl;
    std::cout << "Translation vector size : " << params.t_vecs.size() << std::endl;
    return params;
}


StereoCamParams CameraCalibrator::run(CalibConfig &cfg) {
    cv::Mat im1, frame1, im2, frame2;
    std::vector<cv::Point2f> corner_pts1, corner_pts2;
    std::vector<std::vector<cv::Point3f>> obj_pts1, obj_pts2;
    std::vector<std::vector<cv::Point2f>> image_pts1, image_pts2;
    StereoCamParams params;
    bool s1, s2;

    cv::glob(cfg.cam1.dir_pattern, cfg.cam1.filenames);
    cv::glob(cfg.cam2.dir_pattern, cfg.cam2.filenames);

    if((cfg.cam1.filenames.size() == 0 || cfg.cam2.filenames.size() == 0) || (cfg.cam1.filenames.size() != cfg.cam2.filenames.size())) {
        ERRORLOG("Error loading files in directory, exitting");
        exit(-1);
    }

    std::vector<cv::Point3f> objp;
    for(int i{0}; i<checkerboard_size.second; i++) {
        for(int j{0}; j<checkerboard_size.first; j++)
        objp.push_back(cv::Point3f(j,i,0));
    }

    INFOLOG("Running calibration for stereo camera with %ld frames and checkerboard dimensions [%d X %d]", cfg.cam1.filenames.size(), checkerboard_size.first, checkerboard_size.second);

    for (int i=0; i<cfg.cam1.filenames.size(); i+=40) {
        im1 = cv::imread(cfg.cam1.filenames[i]);
        im2 = cv::imread(cfg.cam2.filenames[i]);
        cv::cvtColor(im1, frame1, cv::COLOR_BGR2GRAY);
        cv::cvtColor(im2, frame2, cv::COLOR_BGR2GRAY);
        
        INFOLOG("Processing image %s", cfg.cam1.filenames[i]);
        s1 = cv::findChessboardCorners(frame1, cv::Size(checkerboard_size.first, checkerboard_size.second), corner_pts1, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);
        s2 = cv::findChessboardCorners(frame2, cv::Size(checkerboard_size.first, checkerboard_size.second), corner_pts2, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);

        if(s1 && s2) {
            cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001);
            cv::cornerSubPix(frame1, corner_pts1, cv::Size(checkerboard_size.first, checkerboard_size.second), cv::Size(-1,-1),criteria);
            cv::drawChessboardCorners(im1, cv::Size(checkerboard_size.first, checkerboard_size.second), corner_pts1, s1);
            cv::cornerSubPix(frame2, corner_pts2, cv::Size(checkerboard_size.first, checkerboard_size.second), cv::Size(-1,-1),criteria);
            cv::drawChessboardCorners(im2, cv::Size(checkerboard_size.first, checkerboard_size.second), corner_pts2, s2);
            obj_pts1.push_back(objp);
            obj_pts2.push_back(objp);

            image_pts1.push_back(corner_pts1);
            image_pts2.push_back(corner_pts2);
        }
        else {
            WARNLOG("No checkerboard detected in view");
        }
        cv::Mat joinedim;
        cv::hconcat(im1, im2, joinedim);
        cv::imshow("Image", joinedim);
        int key = (cv::waitKey(1) & 0xFF);
        if(key == 'q') {
            INFOLOG("Caught 'q' keypress, exitting process.");
            cv::destroyAllWindows();
            exit(0);
        }
    }
    INFOLOG("Iterative point detection complete, computing intrinsic and extrinsic camera matrices...");

    cv::destroyAllWindows();

    INFOLOG("Object Points : %ld, Image Points : %ld.", obj_pts1.size(), image_pts1.size());
    INFOLOG("Running mono compute for camera 1 ...");
    cv::calibrateCamera(obj_pts1, image_pts1, cv::Size(frame1.rows, frame1.cols), params.cam1_params.K, params.cam1_params.dist_coeff, params.cam1_params.R_vecs, params.cam1_params.t_vecs);
    INFOLOG("Done!");
    INFOLOG("Running mono compute for camera 2 ...");
    cv::calibrateCamera(obj_pts1, image_pts2, cv::Size(frame1.rows, frame1.cols), params.cam2_params.K, params.cam2_params.dist_coeff, params.cam2_params.R_vecs, params.cam2_params.t_vecs);
    INFOLOG("Done!")
    INFOLOG("Starting stereoscopic calibration compute...");
    cv::stereoCalibrate(
        obj_pts1, 
        image_pts1, 
        image_pts2, 
        params.cam1_params.K, params.cam1_params.dist_coeff, 
        params.cam2_params.K, params.cam2_params.dist_coeff, 
        cv::Size(frame1.rows, frame1.cols), 
        params.Rs, params.ts, 
        params.E, params.F
    );
    INFOLOG("Done!");

    std::cout << "cameraMatrix 1 : " << params.cam1_params.K << std::endl;
    std::cout << "distCoeffs 1 : " << params.cam1_params.dist_coeff << std::endl;
    std::cout << "cameraMatrix 2 : " << params.cam2_params.K << std::endl;    
    std::cout << "distCoeffs 2 : " << params.cam2_params.dist_coeff << std::endl;
    return params;
}