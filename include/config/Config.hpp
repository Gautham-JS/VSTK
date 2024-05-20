#include <stdio.h>
#include <string>
#include <utility>

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

#ifndef __VSTK_CONFIG_H_
#define __VSTK_CONFIG_H_

namespace vstk {
    enum class LoadScheme {
        IN_MEMORY,
        DISK
    };

    enum class RunType {
        STATIC_DIR,
        STREAM
    };

    enum SLAMType {
        STEREO,
        MONO,
        RGBD
    };

    enum class MatchAlgorithm {
        BF,
        BF_HAMMING,
        FLANN
    };

    enum class FExtractionAlgorithm {
        ORB,
        FAST,
        ADAPTIVE_FAST,
        SIFT,
        ORB_SLAM_ORB
    };

    enum class DComputeAlgorithm {
        ORB,
        SIFT,
        BRIEF
    };

    enum CFG_LOAD_RETURN_CODES {
        OK = 0,
        ERROR_GENERIC = 1,
        ERROR_FORMAT = 2,
        ERROR_FILE_UNDEF = 3,
        ERROR_PROP_UNDEF = 4,
    };

    class MonoCamParams {
        public:
            cv::Mat K, dist_coeff;
            std::vector<cv::Mat> R_vecs, t_vecs;
            int n_rows, n_cols;
    };

    class StereoCamParams {
        public:
            MonoCamParams cam1_params, cam2_params;
            cv::Mat Rs;
            cv::Mat ts; 
            cv::Mat E;
            cv::Mat F;
            int n_rows, n_cols;
    };

    int read_stereo_params(std::string filepath, StereoCamParams &params);


    
    class VstkConfig {
        private:
            int num_features_retained = 5000;
            RunType run_type;
            LoadScheme load_scheme;
            SLAMType slam_type;

            FExtractionAlgorithm feature_extraction_algo = FExtractionAlgorithm::FAST;
            DComputeAlgorithm descriptor_compute_algo = DComputeAlgorithm::ORB;
            MatchAlgorithm match_algo = MatchAlgorithm::BF;

            std::shared_ptr<StereoCamParams> stereo_cam_params = nullptr;
            std::shared_ptr<MonoCamParams> mono_cam_params = nullptr;

            std::string run_data_source, stereo_im1_source, stereo_im2_source;
            std::string working_dir;
            std::string config_file;

            // Adaptive FAST algorithm parameters.
            int min_count_adafast = 200;
            int max_count_adafast = 2000;
            int min_threshold_adafast = 10;
            int max_threshold_adafast = 100;
            int threshold_step_size_adafast = 10;
            std::pair<int, int> cell_size_adafast = std::make_pair(3, 3);

            template<typename T>
            inline int set_cfg(cv::FileNode node, T &property) {
                int rc = ERROR_PROP_UNDEF;
                if(!node.empty()) { 
                    node >> property;
                    rc = OK;
                }
                return rc;
            }


        public:
            explicit VstkConfig(RunType run_type, std::string working_dir);
            VstkConfig(RunType run_type, std::string working_dir, LoadScheme loading_scheme);
            VstkConfig();

            // Config file loading functions
            int load_from_yaml(std::string filepath);
            int load_adafast_properties(cv::FileNode node);
            int load_detector_properties(cv::FileNode node);
            int load_camera_properties(cv::FileNode node);

            void set_working_dir(std::string working_dir);
            void set_run_data_src(std::string run_data_dir);
            void set_load_scheme(LoadScheme load_scheme);
            void set_run_type(RunType run_type);
            void set_feature_extraction_algo(FExtractionAlgorithm algorithm);
            void set_descriptor_compute_algo(DComputeAlgorithm algorithm);
            void set_match_algo(MatchAlgorithm algorithm);
            void set_num_features_retained(int num_features);
            void set_slam_type(SLAMType type);
            void set_stereo_cam_params(std::shared_ptr<StereoCamParams> stereo_params);
            void set_mono_cam_params(std::shared_ptr<MonoCamParams> mono_params);

            RunType get_run_type();
            LoadScheme get_load_scheme();
            std::string get_working_dir();
            std::string get_run_data_src();
            int get_num_features_retained();
            inline std::string get_stereo_src_1() {
              return this->stereo_im1_source;
            }

            inline std::string get_stereo_src_2() {
              return this->stereo_im1_source;
            }

            

            bool is_config_file_used();
            
            // Configurations for custom Adaptive FAST extractor algorithm (ADA-FAST)
            int get_min_count_adafast();
            int get_max_count_adafast();
            int get_min_threshold_adafast();
            int get_max_threshold_adafast();
            int get_threshold_step_size_adafast();
            std::pair<int, int> get_cell_size_adafast();

            // Configurations for RANSAC based F matrix compute
            int get_ransac_reprojection_threshold();
            int get_ransac_confidence_score();


            SLAMType get_slam_type();
            FExtractionAlgorithm get_feature_extraction_algo();
            DComputeAlgorithm get_descriptor_compute_algo();
            MatchAlgorithm get_match_algorithm();

            std::shared_ptr<StereoCamParams> get_stereo_cam_params();
            std::shared_ptr<MonoCamParams> get_mono_cam_params();
    };

    std::string enum_to_str(FExtractionAlgorithm algo);

    std::string enum_to_str(DComputeAlgorithm algo);

    std::string enum_to_str(MatchAlgorithm algo);

    FExtractionAlgorithm extractor_algo_str_to_enum(std::string f_algorithm);
    MatchAlgorithm matcher_algo_str_to_enum(std::string m_algorithm);
    DComputeAlgorithm desc_compute_algo_str_to_enum(std::string d_algorithm);
}

#endif
