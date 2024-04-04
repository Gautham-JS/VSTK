#include "features/FeatureExtractor.hpp"
#include "utils/Logger.hpp"



x3ds::ImageContextHolder::ImageContextHolder(std::string image_path) {
    ImageContextHolder(image_path, LoadScheme::IN_MEMORY_COMPLETE);
}

x3ds::ImageContextHolder::ImageContextHolder(std::string image_path, x3ds::LoadScheme load_scheme) 
    : image_path(image_path), loading_scheme(load_scheme) {
    
    cv::Mat image = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
    x3ds::FeatureExtractor extractor(
        std::move(std::make_shared<cv::Mat>(image)),
        load_scheme
    );
    this->feature_extractor = std::make_shared<FeatureExtractor>(extractor);
}

x3ds::FeatureExtractor x3ds::ImageContextHolder::get_extractor() {
    return *(this->feature_extractor.get());
}

cv::Mat x3ds::ImageContextHolder::get_image() {
    return this->get_extractor().get_image();
}

cv::Mat x3ds::ImageContextHolder::get_descriptors() {
    return this->get_extractor().get_descriptors();
}

std::vector<cv::KeyPoint> x3ds::ImageContextHolder::get_keypoints() {
    return this->get_extractor().get_keypoints();
}

std::string x3ds::ImageContextHolder::get_image_path() {
    return this->image_path;
}

x3ds::FeatureExtractor::FeatureExtractor(std::shared_ptr<cv::Mat> &&imagePtr) : image(std::move(imagePtr)) {}
x3ds::FeatureExtractor::FeatureExtractor(std::shared_ptr<cv::Mat> &&imagePtr, x3ds::LoadScheme load_scheme) 
    : image(std::move(imagePtr)), load_scheme(load_scheme) {}


void x3ds::FeatureExtractor::run_sift() {
    INFOLOG("Running SIFT feature detector \n");
    this->f2d->detect(*(this->image.get()), this->kps);
    this->f2d->compute(*(this->image.get()), this->kps, this->descriptors);
}


void x3ds::FeatureExtractor::display_features() {
    cv::Mat kp_im;
    cv::drawKeypoints(*(this->image.get()), this->kps, kp_im, cv::Scalar(0, 255, 0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::imshow("SIFT keypoints", kp_im);
    cv::waitKey();
}


std::vector<cv::KeyPoint> x3ds::FeatureExtractor::get_keypoints() {
    return this->kps;
}

cv::Mat x3ds::FeatureExtractor::get_descriptors() {
    return this->descriptors;
}

cv::Mat x3ds::FeatureExtractor::get_image() {
    return *(this->image);
}