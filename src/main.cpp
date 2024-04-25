#include <stdio.h>
#include "opencv2/opencv.hpp"
#include "grpcpp/grpcpp.h"


#include "features/FeatureExtractor.hpp"
#include "features/FeatureMatcher.hpp"
#include "protoimpl/ImageSvc.hpp"
#include "utils/Logger.hpp"
#include "config/Config.hpp"
#include "io/DiskIO.hpp"
#include "io/Serializer.hpp"
#include "utils/TimerUtils.hpp"



using namespace std; 
using namespace vstk;

const std::string working_dir = "/home/gjs/software/vstk/data/";
const std::string im_pattens = "/home/gjs/Documents/data/Freiburg/Large/rgb/*.png";


void load_and_publish_image(std::string path, vstk::VstkConfig conf) {
    FeatureExtractor extractor(conf);
    Serializer serializer;
    DiskIO disk_io(working_dir, "matches");
    vector<string> files = disk_io.list_directory(path);
    for(int i=0; i<files.size(); i++) {
        ImageContextHolder ictx(files[i]);
        extractor.run(ictx);
        vstk::BinaryDataStream *data = serializer.serialize(ictx);
        cerr << i << " : " <<data->data << endl;
    }
}

void run_locally(std::string path) {

    VstkConfig conf;
    conf.set_run_data_dir(path);
    conf.set_num_features_retained(5000);
    conf.set_feature_extraction_algo(vstk::FExtractionAlgorithm::FAST);
    conf.set_descriptor_compute_algo(vstk::DComputeAlgorithm::ORB);
    conf.set_match_algo(vstk::MatchAlgorithm::FLANN);

    DBGLOG(
        "\n=========================================================\nFeature Extraction Algorithm : %s\nDescriptor Compute Algorithm : %s\nFeature Matching Algorithm : %s\n=========================================================\n", 
        vstk::enum_to_str(conf.get_feature_extraction_algo()), 
        vstk::enum_to_str(conf.get_descriptor_compute_algo()),
        vstk::enum_to_str(conf.get_match_algorithm())
    );

    // load_and_publish_image(path, conf);
    // exit(-1);


    DiskIO disk_io(working_dir, "matches");
    vector<string> files = disk_io.list_directory(path);
    vector<ImageContextHolder> image_list;
    
    Timer t_main = get_timer("Main");
    ImageContextHolder image(files[0]);
    FeatureExtractor extractor(conf);
    FeatureMatcher matcher(conf);
    Serializer serializer;

    extractor.run(image);
    DBGLOG("Initial features : %d, Descriptors : %d", image.get_features_holder().kps.size(), image.get_features_holder().descriptors.size());
    image_list.push_back(image);
    int cur_ptr = 1;
    int prev_ptr = 0;

    while (cur_ptr < files.size()) {
        start_timer(t_main);
        INFOLOG("Loading image [ %d / %d ]", cur_ptr, files.size() - 1);
        DBGLOG("Prev PTR : %d, Cur PTR : %d", prev_ptr, cur_ptr);
        
        ImageContextHolder prev_image = image_list[prev_ptr];        
        ImageContextHolder curr_image(files[cur_ptr]);
        extractor.run(curr_image);
        MatchesHolder match_holder = matcher.run(curr_image, prev_image);
        DBGLOG("Detected %d features.", match_holder.good_matches.size());

        if(match_holder.good_matches.size() == 0) {
            WARNLOG("No good quality matches, rejecting image pair");
            cur_ptr++;
            continue;
        }
        matcher.display_match_overlap(curr_image, prev_image, match_holder);
        if(match_holder.good_matches.size() > 40) {
            INFOLOG("[SKIP] Tracking quality way too good, ommiting current frame due to insignificant motion.");
            curr_image.clear_image_data();
        }
        else {
            INFOLOG("[INSERT] Diminishing match quantity, inserting reference image & clearing image %d from memory", prev_ptr);
            image_list[prev_ptr].clear_image_data();
            image_list.emplace_back(curr_image);
            prev_ptr++;
        }
        end_timer(t_main);
        log_fps(t_main, stdout);
        cur_ptr++;
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
    //vstk::Logger::get().enable_debug();
    run_locally(im_pattens);
    //bindToGRPC("localhost:34015");
    return 0;
}
