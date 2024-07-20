#include "features/FeatureMatcher.hpp"
#include "utils/Logger.hpp"
#include "utils/GenericUtils.hpp"
#include "utils/AsyncUtils.hpp"
#include "utils/CvUtils.hpp"

#include <memory.h>
#include <mutex>
#include <thread>
#include <functional>
#include <condition_variable>

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
    if(config.get_descriptor_compute_algo() == vstk::DComputeAlgorithm::ORB && matcher_enum == cv::DescriptorMatcher::FLANNBASED) {
        matcher = cv::makePtr<cv::FlannBasedMatcher>(
            cv::FlannBasedMatcher(
                cv::makePtr<cv::flann::LshIndexParams>(12, 20, 2)
            )
        );
    }
    else{
        matcher = cv::DescriptorMatcher::create(matcher_enum);
    }
}

void FeatureMatcher::lowe_threshold(MatchesHolder &holder) {
    const float ratio = 0.7f;
    std::vector<cv::DMatch> good_matches;
    for(int i=0; i<holder.knn_matches.size(); i++) {
        if(holder.knn_matches[i].size() < 2) {
            continue;
        }
        if (holder.knn_matches[i][0].distance < holder.knn_matches[i][1].distance * ratio) {
            good_matches.emplace_back(holder.knn_matches[i][0]);
        }
    }
    if(good_matches.size() == 0 || holder.knn_matches.size() == 0) {
        ERRORLOG("No matches/good matches found!");
    }
    double good_pct = ((good_matches.size() / holder.knn_matches.size()) * 100);
    DBGLOG("[Lowe's Ratio Filter] Original Matches : %ld | Filtered Matches : %ld | Filtration Percentage : %f", holder.knn_matches.size(), good_matches.size(), good_pct);
    holder.good_matches = good_matches;
}


MatchesHolder FeatureMatcher::run(ImageContextHolder &image1_ctx, ImageContextHolder &image2_ctx) {
    DBGLOG("Running matcher for images");
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
    filter_ransac(holder, image1_ctx, image2_ctx);
    return holder;
}

MatchesHolder FeatureMatcher::run_symmetric(ImageContextHolder &image1_ctx, ImageContextHolder &image2_ctx) {
    DBGLOG("Running symmetric matcher for images\n");
    MatchesHolder holder12, holder21, symmetric_holder;
    std::mutex mtx;
    bool ready = false;
    std::condition_variable cvar;
    std::thread t1{
        [&image1_ctx, &image2_ctx, this, &holder12] {
            std::unique_lock<std::mutex>(mtx);
            this->matcher->knnMatch(
                image1_ctx.get_features_holder().descriptors,
                image2_ctx.get_features_holder().descriptors,
                holder12.knn_matches,
                2
            );
            lowe_threshold(holder12);
        }
    };

    std::thread t2{
        [&image1_ctx, &image2_ctx, this, &holder21] {
            std::unique_lock<std::mutex>(mtx);
            this->matcher->knnMatch(
                image2_ctx.get_features_holder().descriptors,
                image1_ctx.get_features_holder().descriptors,
                holder21.knn_matches,
                2
            );
            lowe_threshold(holder21);
        }
    };

    t1.join();
    t2.join();

    this->symmetry_test(holder12.good_matches, holder21.good_matches, symmetric_holder.good_matches);
    return symmetric_holder;
}


void FeatureMatcher::display_match_overlap(ImageContextHolder current_image, ImageContextHolder prev_image, MatchesHolder holder) {
    cv::Mat joined;
    cv::Mat img = current_image.get_image().clone();
    cv::Mat img_delta = img.clone();
    //cv::drawKeypoints(img, current_image.get_features_holder().kps, img, cv::Scalar(255, 0, 0), cv::DrawMatchesFlags::DEFAULT);
    for(int i=0; i<holder.good_matches.size(); i++) {
        cv::DMatch match_pt = holder.good_matches[i];
        int ic = match_pt.queryIdx;
        int ip = match_pt.trainIdx;
        cv::KeyPoint kpc = current_image.get_features_holder().kps[ic];
        cv::KeyPoint kpp = prev_image.get_features_holder().kps[ip];
        cv::drawMarker(img, kpc.pt, cv::Scalar(0, 255, 0), cv::MarkerTypes::MARKER_CROSS, 20, 1);

        cv::drawMarker(img_delta, kpc.pt, cv::Scalar(0, 255, 0), cv::MarkerTypes::MARKER_TILTED_CROSS, 20, 1);
        cv::drawMarker(img_delta, kpp.pt, cv::Scalar(255, 0, 0), cv::MarkerTypes::MARKER_TILTED_CROSS, 20, 1);
        cv::line(img_delta, kpc.pt, kpp.pt, cv::Scalar(0, 255, 255), 2, cv::LineTypes::LINE_AA);
    }
    cv::Mat curr_image_data = current_image.get_image();
    cv::putText(curr_image_data, "Original Image", cv::Point(0, 0), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1);
    cv::vconcat(curr_image_data, img, img);
    cv::vconcat(img, img_delta ,joined);
    cv::resize(joined, joined, {joined.cols/2, joined.rows/2});
    vstk::imshow("FLANN Matches", joined);
    int key = (cv::waitKey(1) & 0xFF);
    if(key == 'q') {
        INFOLOG("Caught 'q' keypress, exitting process.");
        cv::destroyAllWindows();
        exit(0);
    }
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
    vstk::imshow("FLANN Matches", im_matches);
    int key = (cv::waitKey(1) & 0xFF);
    if(key == 'q') {
        INFOLOG("Caught 'q' keypress, exitting process.");
        cv::destroyAllWindows();
        exit(0);
    }
}

void FeatureMatcher::filter_ransac(MatchesHolder &holder, ImageContextHolder image1_ctx, ImageContextHolder image2_ctx) {
    if( holder.good_matches.size() < 20) {
        WARNLOG("Low match confidence, bypassing RANSAC filter");
        return;
    }
    DBGLOG("Filtering matches with RANSAC");
    uint32_t initial_matches = holder.good_matches.size();
    std::vector<cv::DMatch> filtered_matches;
    std::vector<cv::Point2f> p1, p2;
    for(int i=0; i<holder.good_matches.size(); i++) {
        cv::DMatch match_pt = holder.good_matches[i];

        float x = image1_ctx.get_features_holder().kps[match_pt.queryIdx].pt.x;
        float y = image2_ctx.get_features_holder().kps[match_pt.queryIdx].pt.y;
        p1.emplace_back(cv::Point2f(x, y));

        x = image1_ctx.get_features_holder().kps[match_pt.trainIdx].pt.x;
        y = image2_ctx.get_features_holder().kps[match_pt.trainIdx].pt.y;
        p2.emplace_back(cv::Point2f(x, y));
    }

    std::vector<uchar> inliers(p1.size(),0);
    cv::Mat f_matrix = cv::findFundamentalMat(p1, p2, inliers, cv::RANSAC, 3, 0.99);
    
    auto inlier_iterator = inliers.begin();
    auto match_iterator = holder.good_matches.begin();
    for ( ;inlier_iterator != inliers.end(); ++inlier_iterator, ++match_iterator){
        if (*inlier_iterator) {
            filtered_matches.emplace_back(*match_iterator);
        }
    }
    double filter_percentage = (
        (holder.good_matches.size() - filtered_matches.size()) / (double) holder.good_matches.size()
    ) * 100;
    DBGLOG("[RANSAC Filter] Original Matches : %ld | Filtered Matches : %ld | Filtration Percentage : %f", holder.good_matches.size(), filtered_matches.size(), filter_percentage);
    holder.good_matches = filtered_matches;
}

void FeatureMatcher::filter_homography(MatchesHolder &holder, ImageContextHolder image1_ctx, ImageContextHolder image2_ctx) {
    if(!is_matches_valid(holder)) {
        return;
    }
    DBGLOG("Filtering matches with Homography computation");
    uint32_t initial_matches = holder.good_matches.size();
    std::vector<cv::DMatch> filtered_matches;
    std::vector<cv::Point2f> obj, scene;
    for(int i=0; i<holder.good_matches.size(); i++) {
        cv::DMatch match_pt = holder.good_matches[i];

        cv::Point2f object_pt = image2_ctx.get_features_holder().kps[match_pt.queryIdx].pt;
        cv::Point2f scene_pt = image1_ctx.get_features_holder().kps[match_pt.trainIdx].pt;
        obj.emplace_back(object_pt);
        scene.emplace_back(scene_pt);
    }

    cv::Mat mask;
    int min_inliers = 4;
    double reprojection_error = 5;
    cv::Mat h_matrix = cv::findHomography(obj, scene, cv::FM_RANSAC, reprojection_error, mask);
    int inliers = 0;
    //INFOLOG("Mat rows : %ld, Matches size : %ld", mask.rows, good_matches.size());
    for(int i=0; i<mask.rows; ++i){
        if(mask.at<int>(i) == 1) inliers++;
    }
    DBGLOG("[Homography filter] Inlier count : %d", inliers);

    if(inliers <= min_inliers) {
        DBGLOG("Homographic filter rejected match");
        holder.good_matches.clear();
    }
}

bool FeatureMatcher::is_matches_valid(MatchesHolder holder) {
    return (holder.good_matches.size() >= 8);
}

// void FeatureMatcher::symmetry_test(const std::vector<std::vector<cv::DMatch> >& matches1, const std::vector<std::vector<cv::DMatch> >& matches2, std::vector<cv::DMatch>& symMatches) {
//       // for all matches image 1 -> image 2
//     for (std::vector<std::vector<cv::DMatch> >::const_iterator matchIterator1= matches1.begin(); matchIterator1!= matches1.end(); ++matchIterator1) {
//         // ignore deleted matches
//         if (matchIterator1->size() < 2)
//             continue;
//         // for all matches image 2 -> image 1
//         for (std::vector<std::vector<cv::DMatch> >::const_iterator matchIterator2= matches2.begin(); matchIterator2!= matches2.end(); ++matchIterator2) {
//             // ignore deleted matches
//         if (matchIterator2->size() < 2)
//             continue;
//         // Match symmetry test
//         if ((*matchIterator1)[0].queryIdx == (*matchIterator2)[0].trainIdx &&
//             (*matchIterator2)[0].queryIdx == (*matchIterator1)[0].trainIdx) {
//                 // add symmetrical match
//                 symMatches.push_back(cv::DMatch(
//                     (*matchIterator1)[0].queryIdx,
//                     (*matchIterator1)[0].trainIdx,
//                     (*matchIterator1)[0].distance)
//                 );
//                 break; // next match in image 1 -> image 2
//             }
//         }
//     }
// }


void FeatureMatcher::symmetry_test(std::vector<cv::DMatch> match12, std::vector<cv::DMatch> match21, std::vector<cv::DMatch> &sym_matches) {
    sym_matches.clear();
    for(int i=0; i < match12.size(); i++) {
        cv::DMatch m12 = match12[i];
        for(int j=0; j<match21.size(); j++) {
            cv::DMatch m21 = match21[j];
            if(m12.queryIdx == m21.queryIdx && m12.trainIdx == m12.trainIdx) {
                sym_matches.emplace_back(m12);
            }
        }
    }
}



