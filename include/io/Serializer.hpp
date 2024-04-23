#include <stdio.h>
#include <string>

#include "features/FeatureMatcher.hpp"
#include "features/FeatureExtractor.hpp"

#ifndef __VSTK_SERIALIZER_H
#define __VSTK_SERIALIZER_H

namespace vstk {
    typedef struct BinaryDataStream {
        std::string class_name;
        size_t data_size;
        std::string data;
    } BinaryDataStream;
    
    class Serializer {
        private:
            std::string data_stream_buffer;
            const std::string DELIM = ";";

            std::string stringify_match(const cv::DMatch match);
            std::string stringify_kp(const cv::KeyPoint kp);
            std::string stringify_mat(const cv::Mat matrix);

        public:
            Serializer();
            BinaryDataStream* serialize(ImageContextHolder image_ctx);
            BinaryDataStream* serialize(MatchesHolder match_holder);

    };
}

#endif