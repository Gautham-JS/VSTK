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

void VstkConfig::set_run_data_dir(std::string run_data_dir) {
    this->run_data_directory = run_data_dir;
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

std::string VstkConfig::get_run_data_dir() {
    return this->run_data_directory;
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