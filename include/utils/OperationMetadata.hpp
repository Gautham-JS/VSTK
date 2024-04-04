#include <stdio.h>
#include <vector>
#include <unordered_map>

#include "features/FeatureExtractor.hpp"
#include "features/FeatureMatcher.hpp"

#ifndef __X3DS_OP_METADATA_H
#define __X3DS_OP_METADATA_H

namespace x3ds {
    typedef struct OpMetadata {
        std::vector<std::string> image_id_stream;
        std::unordered_map<std::string, ImageContextHolder> image_data_lookup;
        std::unordered_map<std::string, MatchesHolder> match_data_lookup;
    } MatchesHolder;


    ImageContextHolder getImageContext(OpMetadata metadata, std::string image_id);
    
    MatchesHolder getMatchesHolder(OpMetadata metadata, std::string image_id);
    
    void addImageContext(OpMetadata *metadata, std::string image_id, ImageContextHolder context);
    
    void addMatchesHolder(OpMetadata *metadata, std::string image_id, MatchesHolder matches_holder);
}

#endif