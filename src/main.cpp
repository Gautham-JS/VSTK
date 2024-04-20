#include <stdio.h>
#include "opencv2/opencv.hpp"
#include "grpcpp/grpcpp.h"


#include "features/FeatureExtractor.hpp"
#include "features/FeatureMatcher.hpp"
#include "protoimpl/ImageSvc.hpp"
#include "utils/Logger.hpp"
#include "config/Config.hpp"
#include "io/DiskIO.hpp"



using namespace std; 
using namespace vstk;

const std::string working_dir = "/home/gjs/software/3D_Indexer/data/";
const std::string im_pattens = "/home/gjs/software/vstk/data/freiburg/*.png";

void run_locally(std::string path) {

    VstkConfig conf;
    conf.set_run_data_dir(path);
    conf.set_feature_extraction_algo(vstk::FExtractionAlgorithm::FAST);
    conf.set_descriptor_compute_algo(vstk::DComputeAlgorithm::ORB);
    conf.set_match_algo(vstk::MatchAlgorithm::BF);

    DBGLOG(
        "\n=========================================================\nFeature Extraction Algorithm : %s\nDescriptor Compute Algorithm : %s\nFeature Matching Algorithm : %s\n=========================================================\n", 
        vstk::enum_to_str(conf.get_feature_extraction_algo()), 
        vstk::enum_to_str(conf.get_descriptor_compute_algo()),
        vstk::enum_to_str(conf.get_match_algorithm())
    );


    DiskIO disk_io(working_dir, "matches");
    vector<string> files = disk_io.list_directory(path);
    vector<ImageContextHolder> image_list;
    
    ImageContextHolder image(files[0]);
    FeatureExtractor extractor(conf);
    FeatureMatcher matcher(conf);

    extractor.run_sift(image);
    INFOLOG("Initial features : %d, Descriptors : %d", image.get_features_holder().kps.size(), image.get_features_holder().descriptors.size());
    image_list.push_back(image);

    for (int i=1; i<files.size(); i++) {
        INFOLOG("Loading image [ %d / %d ]", i, files.size() - 1);
        ImageContextHolder prev_image = image_list[i - 1];        
        ImageContextHolder next_image(files[i]);
        extractor.run_sift(next_image);
        
        MatchesHolder match_holder = matcher.run(prev_image, next_image);
        INFOLOG("Detected %d features.", match_holder.good_matches.size());
        matcher.display_matches(prev_image, next_image, match_holder);
        image_list[i - 1].clear_image_data();
        image_list.emplace_back(next_image);
    }
}

void bindToGRPC(std::string addr) {

    INFOLOG("Attempting to bind using grpc to address %s", addr.c_str());
    ImageSvc image_service;
    grpc::ServerBuilder server_builder;
    INFOLOG("Server binding");
    server_builder.AddListeningPort(addr, grpc::InsecureServerCredentials());
    INFOLOG("Registering service");
    server_builder.RegisterService(&image_service);
    INFOLOG("Calling build and start");
    auto server = server_builder.BuildAndStart();
    INFOLOG("Server listening...");

    server->Wait();

    INFOLOG("Shut down server");
}

int main(int argc, char** argv ) {
    run_locally(im_pattens);
    //bindToGRPC("localhost:34015");
    return 0;
}