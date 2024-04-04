#include <string>
#include <vector>
#include "FeatureExtractor.hpp"

#ifndef __CROSS_MATCH_ENGINE
#define __CROSS_MATCH_ENGINE

namespace x3ds {

    enum IM_LOADING_SCHEME {
        IN_MEMORY,
        DISK
    };

    class CrossMatchingEngine {
        private:  
            std::vector<x3ds::ImageContextHolder> image_stream;
            std::string images_file_pattern;
            IM_LOADING_SCHEME load_scheme;
        public:
            explicit CrossMatchingEngine(std::string images_path);
            explicit CrossMatchingEngine(std::string images_path, IM_LOADING_SCHEME load_scheme);
    };
}

#endif



