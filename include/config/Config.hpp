#include <stdio.h>
#include <string>

#ifndef __VSTK_CONFIG_H_
#define __VSTK_CONFIG_H_

namespace vstk {
    enum class LoadScheme {
        IN_MEMORY,
        DISK
    };

    enum class RunType {
        STATIC_DIR,
        RPC_SERVICE
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

    
    class VstkConfig {
        private:
            int num_features_retained = 5000;
            RunType run_type;
            LoadScheme load_scheme;

            FExtractionAlgorithm feature_extraction_algo = FExtractionAlgorithm::FAST;
            DComputeAlgorithm descriptor_compute_algo = DComputeAlgorithm::ORB;
            MatchAlgorithm match_algo = MatchAlgorithm::BF;

            std::string run_data_directory;
            std::string working_dir;


        public:
            explicit VstkConfig(RunType run_type, std::string working_dir);
            VstkConfig(RunType run_type, std::string working_dir, LoadScheme loading_scheme);
            VstkConfig();

            void set_working_dir(std::string working_dir);
            void set_run_data_dir(std::string run_data_dir);
            void set_load_scheme(LoadScheme load_scheme);
            void set_run_type(RunType run_type);
            void set_feature_extraction_algo(FExtractionAlgorithm algorithm);
            void set_descriptor_compute_algo(DComputeAlgorithm algorithm);
            void set_match_algo(MatchAlgorithm algorithm);
            void set_num_features_retained(int num_features);

            RunType get_run_type();
            LoadScheme get_load_scheme();
            std::string get_working_dir();
            std::string get_run_data_dir();
            int get_num_features_retained();

            FExtractionAlgorithm get_feature_extraction_algo();
            DComputeAlgorithm get_descriptor_compute_algo();
            MatchAlgorithm get_match_algorithm();
    };

    std::string enum_to_str(FExtractionAlgorithm algo);

    std::string enum_to_str(DComputeAlgorithm algo);

    std::string enum_to_str(MatchAlgorithm algo);
}

#endif
