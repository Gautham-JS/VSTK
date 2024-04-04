#include <opencv2/opencv.hpp>
#include <memory.h>
#include <string.h>

#include "config/config.hpp"

#ifndef __FEATURE_EXTRACTOR_H
#define __FEATURE_EXTRACTOR_H

/**
 * ImageContextHolder is a basic abstraction of the image metadata and the image itself, The feature extractor is contained within the Image Context.
 * 
 * Need to define Loader Interface for each of the types of image inputs:
 *      ->  (DISK):  
 *          Does not load images to memory unless required for processing by some downstream class, immediately cleared 
 *          from memory once done. : Saves memory but slower due to disk IO speed.
 * 
 *      ->  (IN_MEMORY_COMPLETE):
 *          Loads the complete image data onto memory, image data available in memory at all times : Consistently high and 
 *          linearly scaling memory model that is protortional to the image sizes. Fastest in terms of speed. Ideal for small image sets.
 * 
 *      ->  (IN_MEMORY_BATCHED):
 *          Needa think on this a bit more on this design. Intuition is to maintain a batch of images in memory 
 *          to facilitate parallel processing between batches. Images in a batch should be batches based on some nearest-neighbour index (matches/kps)    
 *          
*/


namespace x3ds {

    class FeatureExtractor {
        private:
            cv::Ptr<cv::Feature2D> f2d = cv::SIFT::create();
            std::vector<cv::KeyPoint> kps;
            cv::Mat descriptors;
            std::shared_ptr<cv::Mat> image;
            LoadScheme load_scheme;

        public:
            explicit FeatureExtractor(std::shared_ptr<cv::Mat> &&imagePtr);
            explicit FeatureExtractor(std::shared_ptr<cv::Mat> &&imagePtr, LoadScheme load_scheme);
            explicit FeatureExtractor();
            void run_sift();
            void display_features();
            std::vector<cv::KeyPoint> get_keypoints();
            cv::Mat get_descriptors();
            cv::Mat get_image();
    };

    class ImageContextHolder {
        private:
            std::shared_ptr<x3ds::FeatureExtractor> feature_extractor;
            std::string image_path;
            LoadScheme loading_scheme;

            void init();
            x3ds::FeatureExtractor get_extractor();
        public:
            explicit ImageContextHolder(std::string image_path);
            explicit ImageContextHolder(std::string image_path, LoadScheme load_scheme);
            std::string get_image_path();
            cv::Mat get_image();
            std::vector<cv::KeyPoint> get_keypoints();
            cv::Mat get_descriptors();
    };
}

#endif
