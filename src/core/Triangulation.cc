#include "core/Triangulation.hpp"
#include "utils/CvUtils.hpp"

using namespace vstk;

StereoTriangulate::StereoTriangulate(VstkConfig conf) : conf(conf) {}

CamView StereoTriangulate::run_sparse(ImageContextHolder l_im, ImageContextHolder r_im, MatchesHolder match_holder) {
  INFOLOG("Running sparse stereo triangulation on image pair [%s] & [%s]",l_im.get_image_id(), r_im.get_image_id());
  CamView camera_view;
  std::vector<cv::Point2f> l_kps, l_kps_undist;
  std::vector<cv::Point2f> r_kps, r_kps_undist;     
  cv::Mat Rs = conf.get_stereo_cam_params()->Rs;                                            // Rotation mat between cam1 & cam2
  cv::Mat ts = conf.get_stereo_cam_params()->ts;                                            // Translation vector between cam1 & cam2
  cv::Mat l_P = cv::Mat(3, 4, Rs.type(), cv::Scalar(0));                                                 // Camera pair Projection Matrix
  cv::Mat r_P = cv::Mat(3, 4, Rs.type(), cv::Scalar(0));
  cv::Mat triang_pts_4D;                                                                    // un-normalized 4D points 

  cv::Mat R_ref(3, 3, CV_64F, cv::Scalar(0));
  R_ref.at<float>(0, 0) = 1.0f;
  R_ref.at<float>(1, 1) = 1.0f;
  R_ref.at<float>(2, 2) = 1.0f;

  cv::Mat t_ref(1, 3, CV_64F, cv::Scalar(0));

  for(cv::DMatch match_pt : match_holder.good_matches) {
    float l_x = l_im.get_features_holder().kps[match_pt.queryIdx].pt.x;
    float l_y = l_im.get_features_holder().kps[match_pt.queryIdx].pt.y;
    float r_x = r_im.get_features_holder().kps[match_pt.trainIdx].pt.x;
    float r_y = r_im.get_features_holder().kps[match_pt.trainIdx].pt.y;

    DBGLOG("2D pt coord : (%f, %f)", r_x, r_y);

    l_kps.push_back(cv::Point2f(l_x, l_y));
    r_kps.push_back(cv::Point2f(r_x, r_y));
  }



  DBGLOG("Undistorting points...");
  // TODO : Parallelize this
  cv::undistortPoints(
    l_kps,
    l_kps_undist, 
    conf.get_stereo_cam_params()->cam1_params.K, 
    conf.get_stereo_cam_params()->cam1_params.dist_coeff
  );
  cv::undistortPoints(
    r_kps,
    r_kps_undist, 
    conf.get_stereo_cam_params()->cam2_params.K, 
    conf.get_stereo_cam_params()->cam2_params.dist_coeff
  );
  DBGLOG("Undistortion complete")
  DBGLOG("Constructing Projection matrix...");
  // TODO : parallelize this
  
  for(int r=0; r<Rs.rows; r++) {
    for(int c=0; c<Rs.cols; c++) {
      l_P.at<double>(r, c) = Rs.at<double>(r, c);
    }
  }

  for(int r=0; r<Rs.rows; r++) {
    for(int c=0; c<Rs.cols; c++) {
      r_P.at<double>(r, c) = Rs.at<double>(r, c);
    }
  }
  r_P.at<double>(0, 3) = -1 * ts.at<double>(0, 0);
  // r_P.at<double>(1, 3) = ts.at<double>(1, 0);
  // r_P.at<double>(2, 3) = ts.at<double>(2, 0);

  std::cerr << r_P << std::endl;

  l_P = conf.get_stereo_cam_params()->cam1_params.K * l_P;
  r_P = conf.get_stereo_cam_params()->cam2_params.K * r_P;
  
  DBGLOG("Projection matrix constructed successfully");
  DBGLOG("Starting triangulation process for 4D point coordinates...");
  cv::triangulatePoints(l_P, r_P, l_kps_undist, r_kps_undist, triang_pts_4D);  
  DBGLOG("Triangulation complete, estimated %ld 4D points in space", triang_pts_4D.cols);
  DBGLOG("Normalizing 4D space into 3D space in camera coordinate system...");
  
  camera_view.pts3d.reserve(triang_pts_4D.cols);
  for(size_t i=0; i<triang_pts_4D.cols; i++){
    cv::Point3f localpt;
    localpt.x = triang_pts_4D.at<float>(0,i) / triang_pts_4D.at<float>(3,i);
    localpt.y = triang_pts_4D.at<float>(1,i) / triang_pts_4D.at<float>(3,i);
    localpt.z = triang_pts_4D.at<float>(2,i) / triang_pts_4D.at<float>(3,i);
   //std::cerr << localpt << std::endl;
    camera_view.pts3d.emplace_back(localpt);
  }
  std::cerr<< triang_pts_4D.t() << std::endl;
  vstk::write_ply(camera_view.pts3d, "./scene.ply");
  camera_view.Rc = Rs;
  camera_view.tc = ts;
  return camera_view;
}
