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
        void filter_ransac(MatchesHolder &holder, ImageContextHolder image1_ctx, ImageContextHolder image2_ctx);
        void filter_homography(MatchesHolder &holder, ImageContextHolder image1_ctx, ImageContextHolder image2_ctx);
        void symmetry_test(std::vector<cv::DMatch> match12, std::vector<cv::DMatch> match21, std::vector<cv::DMatch> &sym_matches);
        bool is_matches_valid(MatchesHolder holder);
    public:
        FeatureMatcher(vstk::VstkConfig config);
        MatchesHolder run(ImageContextHolder &image1_ctx, ImageContextHolder &image2_ctx);
        MatchesHolder run_symmetric(ImageContextHolder &image1_ctx, ImageContextHolder &image2_ctx);
        void display_matches(ImageContextHolder image1_ctx, ImageContextHolder image2_ctx, MatchesHolder holder);
        void display_match_overlap(ImageContextHolder current_image, ImageContextHolder prev_image, MatchesHolder holder);
    };
    
}

#endif