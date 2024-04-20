#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/features2d.hpp>
#include <memory.h>

#include "features/FeatureExtractor.hpp"
#include "utils/ObjectLifecycle.hpp"

#ifndef __FEATURE_MATCHER_H
#define __FEATURE_MATCHER_H

namespace vstk {

    typedef struct MatchesHolder {
        std::vector<std::vector<cv::DMatch>> knn_matches;
        std::vector<cv::DMatch> good_matches;
        std::string im1_id;
        std::string im2_id;
    } MatchesHolder;

    class FeatureMatcher{
    private:
        cv::Ptr<cv::DescriptorMatcher> matcher;
        vstk::VstkConfig config;

        void lowe_threshold(MatchesHolder &holder);
    public:
        FeatureMatcher(vstk::VstkConfig config);
        MatchesHolder run(ImageContextHolder &image1_ctx, ImageContextHolder &image2_ctx);
        void display_matches(ImageContextHolder image1_ctx, ImageContextHolder image2_ctx, MatchesHolder holder);
    };
    
}

#endif
