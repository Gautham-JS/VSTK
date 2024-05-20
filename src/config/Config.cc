#include "config/Config.hpp"

using namespace vstk;


VstkConfig::VstkConfig() {}

VstkConfig::VstkConfig(RunType run_type, std::string working_dir) : 
    run_type(run_type),
    working_dir(working_dir) {}

VstkConfig::VstkConfig(RunType run_type, std::string working_dir, LoadScheme loading_scheme) :
    load_scheme(load_scheme) { VstkConfig(run_type, working_dir); }

void VstkConfig::set_working_dir(std::string working_dir) {
    this->working_dir = working_dir;
}

void VstkConfig::set_run_data_src(std::string run_data_dir) {
    this->run_data_source = run_data_dir;
}

void VstkConfig::set_load_scheme(LoadScheme load_scheme) {
    this->load_scheme = load_scheme;
}

void VstkConfig::set_run_type(RunType run_type) {
    this->run_type = run_type;
}

RunType VstkConfig::get_run_type() {
    return this->run_type;
}

LoadScheme VstkConfig::get_load_scheme() {
    return this->load_scheme;
}

std::string VstkConfig::get_working_dir() {
    return this->working_dir;
}

std::string VstkConfig::get_run_data_src() {
    return this->run_data_source;
}


void VstkConfig::set_feature_extraction_algo(FExtractionAlgorithm algorithm) {
    this->feature_extraction_algo = algorithm;
}

void VstkConfig::set_descriptor_compute_algo(DComputeAlgorithm algorithm) {
    this->descriptor_compute_algo = algorithm;
}

void VstkConfig::set_match_algo(MatchAlgorithm algorithm) {
    this->match_algo = algorithm;
}

FExtractionAlgorithm VstkConfig::get_feature_extraction_algo() {
    return this->feature_extraction_algo;
}

DComputeAlgorithm VstkConfig::get_descriptor_compute_algo() {
    return this->descriptor_compute_algo;
}

MatchAlgorithm VstkConfig::get_match_algorithm() {
    return this->match_algo;
}

void vstk::VstkConfig::set_num_features_retained(int num_features) {
    this->num_features_retained = num_features;
}

int vstk::VstkConfig::get_num_features_retained() {
    return this->num_features_retained;
}

void vstk::VstkConfig::set_slam_type(vstk::SLAMType type) {
    this->slam_type = slam_type;
}

SLAMType VstkConfig::get_slam_type() {
    return this->slam_type;
}


int VstkConfig::get_min_count_adafast() {
    return this->min_count_adafast;
}

int VstkConfig::get_max_count_adafast() {
    return this->max_count_adafast;
}

int VstkConfig::get_min_threshold_adafast() {
    return this->min_threshold_adafast;
}

int VstkConfig::get_max_threshold_adafast() {
    return this->max_threshold_adafast;
}

int VstkConfig::get_threshold_step_size_adafast() {
    return this->threshold_step_size_adafast;
}

std::pair<int, int> VstkConfig::get_cell_size_adafast() {
    return this->cell_size_adafast;
}


int VstkConfig::load_from_yaml(std::string filename) {
    int rc = 1;
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    cv::FileNode node;

    set_cfg<std::string>(fs["directory_pattern_l"], this->stereo_im1_source);
    set_cfg<std::string>(fs["directory_pattern_r"], this->stereo_im2_source);
    

    rc = load_adafast_properties(fs["adafast"]);
    rc = load_detector_properties(fs["sparse_detector"]);
    rc = load_camera_properties(fs["cam_params"]);
    return rc;
}

int VstkConfig::load_adafast_properties(cv::FileNode node) {
    int rc = ERROR_GENERIC;
    if(!node.empty()) {
        // configure adafast properties
        rc = this->set_cfg <int> (node["min_count"], this->min_count_adafast);   
        rc = this->set_cfg <int> (node["max_count"], this->max_count_adafast);
        rc = this->set_cfg <int> (node["min_threshold"], this->min_threshold_adafast);
        rc = this->set_cfg <int> (node["max_threshold"], this->max_threshold_adafast);

        int x = cell_size_adafast.first, y = cell_size_adafast.second;
        rc = this->set_cfg <int> (node["cell_size_x"], x);
        rc = this->set_cfg <int> (node["cell_size_y"], y);
        this->cell_size_adafast = std::make_pair(x, y);

        rc = this->set_cfg(node["threshold_step_size"], this->threshold_step_size_adafast);
        rc = 0;
    } 
    return rc;
}

int VstkConfig::load_detector_properties(cv::FileNode node) {
    int rc = ERROR_GENERIC;
    if(!node.empty()) {
        // configure feature detector & matcher properties;
        std::string e_algorithm;
        std::string d_algorithm;
        std::string m_algorithm;

        rc = set_cfg <std::string> (node["feature_extraction_algorithm"], e_algorithm);
        this->set_feature_extraction_algo(extractor_algo_str_to_enum(e_algorithm));

        rc = set_cfg <std::string> (node["descriptor_compute_algorithm"], d_algorithm);
        this->set_descriptor_compute_algo(desc_compute_algo_str_to_enum(d_algorithm));

        rc = set_cfg<std::string> (node["matcher_algorithm"], m_algorithm);
        this->set_match_algo(matcher_algo_str_to_enum(m_algorithm)); 
    }
    return rc;
}

int VstkConfig::load_camera_properties(cv::FileNode node) {
    int rc = ERROR_GENERIC;
    StereoCamParams stereo_params;

    if (!node.empty()) {
        cv::FileNode stereo_props;
        stereo_props = node["stereo_cam_params"];
        if (!stereo_props.empty()) {
            cv::FileNode left_cam, right_cam;
            left_cam = stereo_props["left_cam"];
            right_cam = stereo_props["right_cam"];
            rc = this->set_cfg <cv::Mat> (stereo_props["Rs"], stereo_params.Rs);
            rc = this->set_cfg <cv::Mat> (stereo_props["ts"], stereo_params.ts);
            rc = this->set_cfg <cv::Mat> (stereo_props["FMat"], stereo_params.F);
            rc = this->set_cfg <cv::Mat> (stereo_props["EMat"], stereo_params.E);
            
            if (!left_cam.empty() && !right_cam.empty()) {
                rc = this->set_cfg <cv::Mat> (left_cam["K"], stereo_params.cam1_params.K);
                rc = this->set_cfg <cv::Mat> (left_cam["dist_coeff"], stereo_params.cam1_params.dist_coeff);
                rc = this->set_cfg <cv::Mat> (right_cam["K"], stereo_params.cam2_params.K);
                rc = this->set_cfg <cv::Mat> (right_cam["dist_coeff"], stereo_params.cam2_params.dist_coeff);
                this->set_stereo_cam_params(std::make_shared<StereoCamParams>(stereo_params));  
            }
        }
    }
    return rc;
}



void vstk::VstkConfig::set_stereo_cam_params(std::shared_ptr<StereoCamParams> stereo_params) {
    this->stereo_cam_params = std::move(stereo_params);
}

void vstk::VstkConfig::set_mono_cam_params(std::shared_ptr<MonoCamParams> mono_params) {
    this->mono_cam_params = std::move(mono_params);
}

std::shared_ptr<vstk::StereoCamParams> vstk::VstkConfig::get_stereo_cam_params() {
    return this->stereo_cam_params;
}

std::shared_ptr<vstk::MonoCamParams> vstk::VstkConfig::get_mono_cam_params() {
    return std::move(this->mono_cam_params);
}






int vstk::read_stereo_params(std::string filepath, vstk::StereoCamParams &params) {
    cv::FileStorage fs = cv::FileStorage(filepath, cv::FileStorage::READ);
    if(!fs.isOpened()) {
        return 1;
    }
    fs["Cam1K"] >> params.cam1_params.K;
    fs["Cam1DistCoeffecients"] >> params.cam1_params.dist_coeff;
    fs["Cam2K"] >> params.cam2_params.K;
    fs["Cam2DistCoeffecients"] >> params.cam2_params.dist_coeff;
    fs["Rs"] >> params.Rs;
    fs["ts"] >> params.ts;
    fs["Emat"] >> params.E;
    fs["FMat"] >> params.F;
    fs.release();
    return 0;
}

std::string vstk::enum_to_str(vstk::FExtractionAlgorithm algo) {
    std::string str;
    switch (algo) {
    case FExtractionAlgorithm::ADAPTIVE_FAST :
        str = "ADAPTIVE FAST";
        break;
    case FExtractionAlgorithm::ORB :
        str = "ORB";
        break;
    case FExtractionAlgorithm::FAST :
        str = "FAST";
        break;
    case FExtractionAlgorithm::SIFT :
        str = "SIFT";
        break;
    default:
        str = "Unknown";
        break;
    }
    return str;
}

vstk::FExtractionAlgorithm vstk::extractor_algo_str_to_enum(std::string f_algorithm) {
    FExtractionAlgorithm f_algorithm_enum;
    for (auto & ch: f_algorithm) ch = toupper(ch);
    
    if(f_algorithm == "ADA_FAST" || f_algorithm == "ADAFAST") f_algorithm_enum = vstk::FExtractionAlgorithm::ADAPTIVE_FAST;
    if(f_algorithm == "FAST") f_algorithm_enum = vstk::FExtractionAlgorithm::FAST;
    if(f_algorithm == "ORB") f_algorithm_enum = vstk::FExtractionAlgorithm::ORB;
    if(f_algorithm == "SIFT") f_algorithm_enum = vstk::FExtractionAlgorithm::SIFT;
    return f_algorithm_enum;
}

vstk::MatchAlgorithm vstk::matcher_algo_str_to_enum(std::string m_algorithm) {
    MatchAlgorithm m_algorithm_enum;
    for (auto & ch: m_algorithm) ch = toupper(ch);

    if(m_algorithm == "FLANN")          m_algorithm_enum = vstk::MatchAlgorithm::FLANN;
    if(m_algorithm == "BF")             m_algorithm_enum = vstk::MatchAlgorithm::BF;
    if(m_algorithm == "BF_HAMMING")     m_algorithm_enum = vstk::MatchAlgorithm::BF_HAMMING;
    return m_algorithm_enum;
}

vstk::DComputeAlgorithm vstk::desc_compute_algo_str_to_enum(std::string d_algorithm) {
    DComputeAlgorithm d_compute_algorithm_enum;
    for (auto & ch: d_algorithm) ch = toupper(ch);

    if(d_algorithm == "ORB")            d_compute_algorithm_enum = vstk::DComputeAlgorithm::ORB;
    if(d_algorithm == "BRIEF")          d_compute_algorithm_enum = vstk::DComputeAlgorithm::BRIEF;
    if(d_algorithm == "SIFT")           d_compute_algorithm_enum = vstk::DComputeAlgorithm::SIFT;
    return d_compute_algorithm_enum;
}

std::string vstk::enum_to_str(DComputeAlgorithm algo) {
    std::string str;
    switch (algo) {
    case DComputeAlgorithm::ORB :
        str = "ORB";
        break;
    case DComputeAlgorithm::SIFT :
        str = "SIFT";
        break;
    case DComputeAlgorithm::BRIEF :
        str = "BRIEF";
        break;

    default:
        str = "Unknown";
        break;
    }
    return str;
}

std::string vstk::enum_to_str(MatchAlgorithm algo) {
    std::string str;
    switch (algo) {
    case MatchAlgorithm::BF :
        str = "BRUTE FORCE";
        break;
    case MatchAlgorithm::BF_HAMMING :
        str = "BRUTE FORCE HAMMING";
        break;
    case MatchAlgorithm::FLANN :
        str = "FLANN TREE";
        break;
    default:
        str = "Unknown";
        break;
    }
    return str;
}
