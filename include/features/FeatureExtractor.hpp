#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <memory.h>
#include <string.h>

#include "config/Config.hpp"
#include "utils/Logger.hpp"

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


namespace vstk {

    typedef struct FeaturesHolder {
        std::vector<cv::KeyPoint> kps;
        cv::Mat descriptors;
    } FeaturesHolder;

    typedef struct ImageFeaturePt {
        int num_views;
        cv::KeyPoint kp;
    } ImageFeaturePt;

    class ImageContextHolder {
        private:
            std::string image_id;
            cv::Mat image_data;
            uint32_t image_length;
            uint32_t image_width;
            FeaturesHolder holder;

            void init();
        public:
            explicit ImageContextHolder(std::string image_path);
            explicit ImageContextHolder(unsigned char *image_data, uint32_t image_width, uint32_t image_length);
            explicit ImageContextHolder();
            std::string get_image_id();
            cv::Mat get_image();
            bool is_image_in_memory();
            FeaturesHolder get_features_holder(); 
            void set_feature_holder(FeaturesHolder holder);
            void clear_image_data();

            void load_image_path(std::string image_path);
            void load_image_data(unsigned char *image_data, uint32_t image_length, uint32_t image_width);
    };

    class FeatureExtractor {  
        private:
            cv::Ptr<cv::Feature2D> fextract;
            cv::Ptr<cv::Feature2D> fcompute;
            VstkConfig config;

        public:
            explicit FeatureExtractor(VstkConfig config);
            FeaturesHolder run_sift(ImageContextHolder &image_ctx);
            void display_features(ImageContextHolder image_ctx);
    };
}

#endif
