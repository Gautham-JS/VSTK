#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/features2d.hpp>
#include <memory.h>

#include "features/FeatureExtractor.hpp"
#include "utils/ObjectLifecycle.hpp"

#ifndef __FEATURE_MATCHER_H
#define __FEATURE_MATCHER_H

namespace x3ds {

    typedef struct MatchesHolder {
        std::vector<std::vector<cv::DMatch>> knn_matches;
        std::vector<cv::DMatch> good_matches;
        std::string im1_id;
        std::string im2_id;
    } MatchesHolder;

    class FeatureMatcher{
    private:
        cv::Ptr<cv::DescriptorMatcher> matcher;
        std::shared_ptr<x3ds::FeatureExtractor> im1_extractor, im2_extractor;
        x3ds::ObjectLifecycle lifecycle;
        std::shared_ptr<std::vector<cv::DMatch>> good_matches = nullptr;
        
        void lowe_threshold(std::vector<std::vector<cv::DMatch>> raw_knn_matches);
    public:
        FeatureMatcher(std::unique_ptr<x3ds::FeatureExtractor> &&extractor1, std::unique_ptr<x3ds::FeatureExtractor> &&extractor2);
        FeatureMatcher(std::shared_ptr<x3ds::FeatureExtractor> &&extractor1, std::shared_ptr<x3ds::FeatureExtractor> &&extractor2);
        void run();
        void display_matches();
    };
    
}

#endif
