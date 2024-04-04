#include "features/FeatureMatcher.hpp"
#include "utils/Logger.hpp"

#include <memory.h>


x3ds::FeatureMatcher::FeatureMatcher(std::unique_ptr<x3ds::FeatureExtractor> &&extractor1, std::unique_ptr<x3ds::FeatureExtractor> &&extractor2) : 
    im1_extractor(std::move(extractor1)),
    im2_extractor(std::move(extractor2)),
    lifecycle(std::vector<std::string> {"EMPTY", "MATCHER_COMPLETE"}),
    matcher(cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED)) 
{}

x3ds::FeatureMatcher::FeatureMatcher(std::shared_ptr<x3ds::FeatureExtractor> &&extractor1, std::shared_ptr<x3ds::FeatureExtractor> &&extractor2) : 
    im1_extractor(std::move(extractor1)),
    im2_extractor(std::move(extractor2)),
    lifecycle(std::vector<std::string> {"EMPTY", "MATCHER_COMPLETE"}),
    matcher(cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED)) 
{}

void x3ds::FeatureMatcher::lowe_threshold(const std::vector<std::vector<cv::DMatch>> raw_knn_matches) {
    const float ratio = 0.7f;
    std::vector<cv::DMatch> good_matches;
    for(int i=0; i<raw_knn_matches.size(); i++) {
        if (raw_knn_matches[i][0].distance < raw_knn_matches[i][1].distance * ratio) {
            good_matches.emplace_back(raw_knn_matches[i][0]);
        }
    }
    if(good_matches.size() == 0 || raw_knn_matches.size() == 0) {
        ERRORLOG("No matches/good matches found!");
    }
    double good_pct = ((good_matches.size() / raw_knn_matches.size()) * 100);
    INFOLOG("[Lowe's Ratio Filter] Original Matches : %ld | Filtered Matches : %ld | Filtration Percentage : %f\n", raw_knn_matches.size(), good_matches.size(), good_pct);
    this->good_matches = std::make_unique<std::vector<cv::DMatch>>(good_matches);
}

void x3ds::FeatureMatcher::run() {
    INFOLOG("Running FLANN based matcher for images\n");
    std::vector<std::vector<cv::DMatch>> knn_matches;
    this->matcher->knnMatch(
        this->im1_extractor.get()->get_descriptors(),
        this->im2_extractor.get()->get_descriptors(),
        knn_matches,
        2
    );
    lowe_threshold(knn_matches);
}


void x3ds::FeatureMatcher::display_matches() {
    cv::Mat im_matches;
    cv::drawMatches(
        this->im1_extractor->get_image(),
        this->im1_extractor->get_keypoints(),
        this->im2_extractor->get_image(),
        this->im2_extractor->get_keypoints(),
        *(this->good_matches.get()),
        im_matches,
        cv::Scalar(0, 255, 0)
    );
    cv::imshow("FLANN Matches", im_matches);
    cv::waitKey();
}



