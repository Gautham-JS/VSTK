#include "features/FeatureMatcher.hpp"
#include "utils/Logger.hpp"

#include <memory.h>

using namespace vstk;


FeatureMatcher::FeatureMatcher(vstk::VstkConfig config) : config(config) {
    auto matcher_enum = cv::DescriptorMatcher::BRUTEFORCE;
    switch (config.get_match_algorithm()) {
        case vstk::MatchAlgorithm::BF_HAMMING :
            matcher_enum = cv::DescriptorMatcher::BRUTEFORCE_HAMMING;
            break;
        case vstk::MatchAlgorithm::FLANN :
            matcher_enum = cv::DescriptorMatcher::FLANNBASED;
        default:
            break;
    }
    matcher = cv::DescriptorMatcher::create(matcher_enum);
}

void FeatureMatcher::lowe_threshold(MatchesHolder &holder) {
    const float ratio = 0.7f;
    std::vector<cv::DMatch> good_matches;
    for(int i=0; i<holder.knn_matches.size(); i++) {
        if (holder.knn_matches[i][0].distance < holder.knn_matches[i][1].distance * ratio) {
            good_matches.emplace_back(holder.knn_matches[i][0]);
        }
    }
    if(good_matches.size() == 0 || holder.knn_matches.size() == 0) {
        ERRORLOG("No matches/good matches found!");
    }
    double good_pct = ((good_matches.size() / holder.knn_matches.size()) * 100);
    INFOLOG("[Lowe's Ratio Filter] Original Matches : %ld | Filtered Matches : %ld | Filtration Percentage : %f", holder.knn_matches.size(), good_matches.size(), good_pct);
    holder.good_matches = good_matches;
}


MatchesHolder FeatureMatcher::run(ImageContextHolder &image1_ctx, ImageContextHolder &image2_ctx) {
    INFOLOG("Running matcher for images\n");
    MatchesHolder holder;
    holder.im1_id = image1_ctx.get_image_id();
    holder.im2_id = image2_ctx.get_image_id();
    DBGLOG("Desc size in image 1 : %d, Desc size in image 2 : %d", image1_ctx.get_features_holder().descriptors.size(), image2_ctx.get_features_holder().descriptors.size());
    this->matcher->knnMatch(
        image1_ctx.get_features_holder().descriptors,
        image2_ctx.get_features_holder().descriptors,
        holder.knn_matches,
        2
    );
    DBGLOG("Completed FLANN Matching");
    lowe_threshold(holder);
    return holder;
}


void FeatureMatcher::display_matches(ImageContextHolder image1_ctx, ImageContextHolder image2_ctx, MatchesHolder holder) {
    cv::Mat im_matches;
    cv::drawMatches(
        image1_ctx.get_image(),
        image1_ctx.get_features_holder().kps,
        image2_ctx.get_image(),
        image2_ctx.get_features_holder().kps,
        holder.good_matches,
        im_matches,
        cv::Scalar(0, 255, 0)
    );
    cv::imshow("FLANN Matches", im_matches);
    cv::waitKey();
}



