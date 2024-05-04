#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <memory.h>
#include <string.h>
#include <utility>

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

    class AdaptiveFastExtractor {
        private:
            std::pair<int, int> cell_size;
            size_t n_min, n_max;
            size_t nc_min, nc_max;
            int fth_min, fth_max, th_step;
            std::vector<cv::Ptr<cv::Feature2D>> detectors;
            std::vector<int> thresholds;
            void extract_internal(cv::Mat image, std::vector<cv::KeyPoint> &kps, int cell_idx, std::pair<int, int> origin);
            void down_step_threshold(int cell_idx);
            void up_step_threshold(int cell_idx);
        public:
            AdaptiveFastExtractor(VstkConfig config);
            FeaturesHolder extract(ImageContextHolder &image);
    };

    class FeatureExtractor {  
        private:
            cv::Ptr<cv::Feature2D> fextract;
            cv::Ptr<cv::Feature2D> fcompute;
            VstkConfig config;
            uint32_t fast_threshold = 40;
            uint32_t min_features = 200;
            uint32_t max_features = 700;
            uint32_t p_fast_threshold = 40;
            
            AdaptiveFastExtractor *adaptive_extractor;

            

            FeaturesHolder run_internal(ImageContextHolder &image_ctx);

        public:
            explicit FeatureExtractor(VstkConfig config);
            FeaturesHolder run(ImageContextHolder &image_ctx);
            FeaturesHolder run_adaptive(ImageContextHolder &image_ctx, int r_depth);
            void display_features(ImageContextHolder image_ctx);
            void reset();
    };
}

#endif
