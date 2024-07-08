#include <stdio.h>
#include <string>
#include <utility>

#include <stdlib.h>
#include <getopt.h>

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

#include "utils/Logger.hpp"

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

    enum PERSISTENCE_MODES {
        IN_PROCESS_MEMORY = 1,
        REMOTE_IF = 2,
    };

    enum PERSISTENCE_REMOTE_PROVIDER {
        REDIS = 1,
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

    typedef struct RosConfig {
        std::string left_image_topic = "/vstk/cam0/image_raw";
        std::string right_image_topic = "/vstk/cam1/image_raw";
        std::string left_cam_info_topic = "/vstk/cam0/info";
        std::string right_cam_info_topic = "/vstk/cam1/info";
        std::string p3d_topic = "/vstk/core/p3d";
        std::string p2d_topic = "/vstk/core/p2d";
    } RosConfig;


    // struct for storing camera view state
    typedef struct CamView {
        cv::Mat Rc;
        cv::Mat tc;
        std::string view_id;
        std::vector<cv::Point3f> pts3d;
    } CamView;

    // enum for configuring data persistence layer
    typedef struct PersistenceConfig {
        vstk::PERSISTENCE_MODES mode = vstk::PERSISTENCE_MODES::IN_PROCESS_MEMORY;
        std::string rt_id = "";

        vstk::PERSISTENCE_REMOTE_PROVIDER provider = vstk::PERSISTENCE_REMOTE_PROVIDER::REDIS;
        std::string interface_url = "";
    } PersistenceConfig;

    // enum for renderer, stores state of perspective transformation frame
    enum TransformState {
        WORLD       = 1,
        CAMERA      = 2
    };

    // high level struct wrapping all states for renderer thread
    typedef struct RenderState {
        std::string widnow_name = "VSTK";
        int transform_state = TransformState::CAMERA;
        int pt_size = 2;
    } RenderState;

    typedef std::shared_ptr<vstk::StereoCamParams> StereoCamParamsPtr;
    typedef std::shared_ptr<MonoCamParams> MonoCamParamsPtr;
    typedef std::shared_ptr<PersistenceConfig> PersistenceConfigPtr;


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

            StereoCamParamsPtr stereo_cam_params = nullptr;
            MonoCamParamsPtr mono_cam_params = nullptr;
            PersistenceConfigPtr persistence_config = nullptr;

            RosConfig ros_config;

            std::string run_data_source, stereo_im1_source, stereo_im2_source;
            std::string working_dir;
            std::string config_file;

            // Adaptive FAST algorithm parameters.
            int min_count_adafast = 200;
            int max_count_adafast = 2000;
            int min_threshold_adafast = 10;
            int max_threshold_adafast = 100;
            int threshold_step_size_adafast = 10;
            int adafast_thread_count = 4;
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
            void set_persistence_config(std::shared_ptr<PersistenceConfig> persistence_config);

            RunType get_run_type();
            LoadScheme get_load_scheme();
            std::string get_working_dir();
            std::string get_run_data_src();
            int get_num_features_retained();
            
            std::string get_stereo_src_1() {
              return this->stereo_im1_source;
            }

            std::string get_stereo_src_2() {
              return this->stereo_im2_source;
            }

            RosConfig get_ros_config() {
                return this->ros_config;
            }

            bool is_config_file_used();
            
            // Configurations for custom Adaptive FAST extractor algorithm (ADA-FAST)
            int get_min_count_adafast();
            int get_max_count_adafast();
            int get_min_threshold_adafast();
            int get_max_threshold_adafast();
            int get_threshold_step_size_adafast();
            int get_adafast_threadpool_size();
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
            std::shared_ptr<PersistenceConfig> get_persistence_config();
    };

    void describe_config(VstkConfig conf);
    
    VstkConfig build_config_from_args(int argc, char**argv);

    std::string enum_to_str(FExtractionAlgorithm algo);
    std::string enum_to_str(DComputeAlgorithm algo);
    std::string enum_to_str(MatchAlgorithm algo);

    FExtractionAlgorithm extractor_algo_str_to_enum(std::string f_algorithm);
    MatchAlgorithm matcher_algo_str_to_enum(std::string m_algorithm);
    DComputeAlgorithm desc_compute_algo_str_to_enum(std::string d_algorithm);
}

#endif
