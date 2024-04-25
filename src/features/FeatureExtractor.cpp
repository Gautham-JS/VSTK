#include "features/FeatureExtractor.hpp"
#include "utils/Logger.hpp"

#include <functional>
#include <random>
#include <sstream>

using namespace vstk;

static std::random_device               rd;
static std::mt19937                     gen(rd());
static std::uniform_int_distribution<>  dis(0, 15);
static std::uniform_int_distribution<>  dis2(8, 11);

std::string generate_uuid_v4() {
    std::stringstream ss;
    int i;
    ss << std::hex;
    for (i = 0; i < 8; i++) {
        ss << dis(gen);
    }
    ss << "-";
    for (i = 0; i < 4; i++) {
        ss << dis(gen);
    }
    ss << "-4";
    for (i = 0; i < 3; i++) {
        ss << dis(gen);
    }
    ss << "-";
    ss << dis2(gen);
    for (i = 0; i < 3; i++) {
        ss << dis(gen);
    }
    ss << "-";
    for (i = 0; i < 12; i++) {
        ss << dis(gen);
    };
    return ss.str();
}

ImageContextHolder::ImageContextHolder(std::string image_path) {
    load_image_path(image_path);
}

ImageContextHolder::ImageContextHolder(unsigned char *image_data, uint32_t image_width, uint32_t image_length) {
    load_image_data(image_data, image_length, image_width);
}

void ImageContextHolder::load_image_data(unsigned char *image_data, uint32_t image_length, uint32_t image_width) {
    this->image_width = image_width;
    this->image_length = image_length;
    this->image_id = generate_uuid_v4();
    this->image_data = cv::Mat(image_width, image_length, CV_8UC1, image_data);
}

void ImageContextHolder::load_image_path(std::string image_path) {
    this->image_data = cv::imread(image_path, cv::IMREAD_COLOR);
    this->image_id = image_path;
    this->image_length = this->image_data.cols;
    this->image_width = this->image_data.rows;
}

ImageContextHolder::ImageContextHolder() {

}

FeaturesHolder ImageContextHolder::get_features_holder() {
    return this->holder;
}

void ImageContextHolder::set_feature_holder(FeaturesHolder holder) {
    this->holder = holder;
}

cv::Mat ImageContextHolder::get_image() {
    return this->image_data;
}

std::string ImageContextHolder::get_image_id() {
    return this->image_id;
}

bool ImageContextHolder::is_image_in_memory() {
    return this->image_data.empty();
}


void ImageContextHolder::clear_image_data() {
    this->image_data.release();
}


FeaturesHolder FeatureExtractor::run(ImageContextHolder& image_ctx) {
    INFOLOG("Running feature extract and compute engines");
    FeaturesHolder feature_holder;
    this->fextract->detect(
        image_ctx.get_image(), 
        feature_holder.kps
    );
    cv::KeyPointsFilter::retainBest(feature_holder.kps, config.get_num_features_retained());
    this->fcompute->compute(
        image_ctx.get_image(), 
        feature_holder.kps,
        feature_holder.descriptors
    );
    image_ctx.set_feature_holder(feature_holder);
    return feature_holder;
}
void display_features(ImageContextHolder image_ctx);



FeatureExtractor::FeatureExtractor(vstk::VstkConfig config) : config(config) {
    switch (config.get_descriptor_compute_algo()) {
        case vstk::DComputeAlgorithm::ORB :
            this->fcompute = cv::ORB::create();
            break;
        case vstk::DComputeAlgorithm::SIFT :
            this->fcompute = cv::SIFT::create();
            break;
        default:
            this->fcompute = cv::ORB::create();
            break;
    }

    switch (config.get_feature_extraction_algo()) {
        case vstk::FExtractionAlgorithm::FAST :
            this->fextract = cv::FastFeatureDetector::create();
            break;
        case vstk::FExtractionAlgorithm::ORB :
            this->fextract = cv::ORB::create();
            break;
        case vstk::FExtractionAlgorithm::SIFT :
            this->fextract = cv::SIFT::create(500);
        default:
            this->fextract = cv::FastFeatureDetector::create();
            break;
    }
}


void FeatureExtractor::display_features(ImageContextHolder image_ctx) {
    cv::Mat kp_im;
    cv::drawKeypoints(
        image_ctx.get_image(), 
        image_ctx.get_features_holder().kps, 
        kp_im, 
        cv::Scalar(0, 255, 0), 
        cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS
    );
    cv::imshow("keypoints", kp_im);
    cv::waitKey();
}
