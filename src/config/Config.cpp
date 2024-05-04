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
    
    node = fs["adafast"];
    if(!node.empty()) {
        // configure adafast properties
        this->set_cfg <int> (node["min_count"], this->min_count_adafast);   
        this->set_cfg <int> (node["max_count"], this->max_count_adafast);
        this->set_cfg <int> (node["min_threshold"], this->min_threshold_adafast);
        this->set_cfg <int> (node["max_threshold"], this->max_threshold_adafast);

        int x = cell_size_adafast.first, y = cell_size_adafast.second;
        this->set_cfg <int> (node["cell_size_x"], x);
        this->set_cfg <int> (node["cell_size_y"], y);
        this->cell_size_adafast = std::make_pair(x, y);

        this->set_cfg(node["threshold_step_size"], this->threshold_step_size_adafast);
    } 

    

    RETURN_BLOCK:
        return rc;
}


std::string vstk::enum_to_str(vstk::FExtractionAlgorithm algo) {
    std::string str;
    switch (algo) {
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