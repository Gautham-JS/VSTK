#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>

#include "opencv2/opencv.hpp"

#include "features/FeatureExtractor.hpp"
#include "features/FeatureMatcher.hpp"
#include "utils/Logger.hpp"
#include "config/Config.hpp"
#include "io/DiskIO.hpp"
#include "io/Serializer.hpp"
#include "utils/TimerUtils.hpp"

#ifdef VSTK_TRANSPORT_PROTO_GRPC
    #include "grpcpp/grpcpp.h"
    #include "protoimpl/ImageSvc.hpp"
#endif



using namespace std; 
using namespace vstk;

const std::string working_dir = "/home/gjs/software/vstk/data/";
const std::string im_pattens = "/media/gjs/Windows-SSD/Data/TUM/dataset-outdoors3_512_16/mav0/cam0/data/*.png";


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

void run_locally(VstkConfig conf) {

    DBGLOG(
        "\n=========================================================\nFeature Extraction Algorithm : %s\nDescriptor Compute Algorithm : %s\nFeature Matching Algorithm : %s\n=========================================================\n", 
        vstk::enum_to_str(conf.get_feature_extraction_algo()), 
        vstk::enum_to_str(conf.get_descriptor_compute_algo()),
        vstk::enum_to_str(conf.get_match_algorithm())
    );

    // load_and_publish_image(path, conf);
    // exit(-1);


    DiskIO disk_io(working_dir, "matches");
    vector<string> files = disk_io.list_directory(conf.get_run_data_src());
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
    #ifdef VSTK_TRANSPORT_PROTO_GRPC
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
    #else
        ERRORLOG("VSTK is not compiled with gRPC transport layer, exitting.");
        exit(-1);
    #endif
}

void print_usage_and_unalive(char *prog_name) {
    fprintf(stderr, "%s data_source <args>\n", prog_name);
    exit(EXIT_FAILURE);
}


vstk::VstkConfig build_config_from_args(int argc, char**argv) {
    vstk::VstkConfig conf;
    std::string data_src;
    std::string slam_type;
    std::string f_algorithm;
    vstk::FExtractionAlgorithm f_algorithm_enum = vstk::FExtractionAlgorithm::ADAPTIVE_FAST;
    std::string d_algorithm;
    vstk::DComputeAlgorithm d_compute_algorithm_enum = vstk::DComputeAlgorithm::ORB;
    std::string m_algorithm;
    vstk::MatchAlgorithm m_algorithm_enum = vstk::MatchAlgorithm::FLANN;
    std::string config_file_path = "";
    char c;
    opterr = 0;

    while ((c = getopt (argc, argv, "f:d:c:t:m:v")) != -1) {
        switch (c) {
            case 'f':
                f_algorithm = std::string(optarg);
                for (auto & ch: f_algorithm) ch = toupper(ch);

                if(f_algorithm == "ADA_FAST")       f_algorithm_enum = vstk::FExtractionAlgorithm::ADAPTIVE_FAST;
                if(f_algorithm == "FAST")           f_algorithm_enum = vstk::FExtractionAlgorithm::FAST;
                if(f_algorithm == "ORB")            f_algorithm_enum = vstk::FExtractionAlgorithm::ORB;
                if(f_algorithm == "SIFT")           f_algorithm_enum = vstk::FExtractionAlgorithm::SIFT;
                break;

            case 'd':
                d_algorithm = std::string(optarg);
                for (auto & ch: d_algorithm) ch = toupper(ch);

                if(d_algorithm == "ORB")            d_compute_algorithm_enum = vstk::DComputeAlgorithm::ORB;
                if(d_algorithm == "BRIEF")          d_compute_algorithm_enum = vstk::DComputeAlgorithm::BRIEF;
                if(d_algorithm == "SIFT")           d_compute_algorithm_enum = vstk::DComputeAlgorithm::SIFT;
                break;
            
            case 'm':
                m_algorithm = std::string(optarg);
                for (auto & ch: m_algorithm) ch = toupper(ch);

                if(m_algorithm == "FLANN")          m_algorithm_enum = vstk::MatchAlgorithm::FLANN;
                if(m_algorithm == "BF")             m_algorithm_enum = vstk::MatchAlgorithm::BF;
                if(m_algorithm == "BF_HAMMING")     m_algorithm_enum = vstk::MatchAlgorithm::BF_HAMMING;
                break;

            case 'v':
                vstk::Logger::get().enable_debug();
                break;

            case 'c':
                config_file_path = std::string(optarg);
                break;

            default:
                ERRORLOG("Unknwon option");
                abort ();
        }
    }
    
    if(argc - optind < 1) {
        ERRORLOG("Directory not specified");
        print_usage_and_unalive(argv[0]);
    }
    data_src = std::string(argv[optind]);

    if(argc - optind == 2) {
        slam_type = argv[optind];
        for (auto & ch: m_algorithm) ch = toupper(ch);

        if(slam_type == "MONO") conf.set_slam_type(vstk::SLAMType::MONO);
        if(slam_type == "STEREO") conf.set_slam_type(vstk::SLAMType::STEREO);
        if(slam_type == "RGBD") conf.set_slam_type(vstk::SLAMType::RGBD);
    }
    else {
        conf.set_slam_type(vstk::SLAMType::STEREO);
    }
    conf.set_run_data_src(data_src);
    conf.set_feature_extraction_algo(f_algorithm_enum);
    conf.set_descriptor_compute_algo(d_compute_algorithm_enum);
    conf.set_working_dir(working_dir);
    conf.set_match_algo(m_algorithm_enum);

    return conf;
}

int main(int argc, char** argv ) {

    int aflag = 0;
    int bflag = 0;
    char *cvalue = NULL;
    int index;
    int c;
    vstk::VstkConfig conf = build_config_from_args(argc, argv);
    run_locally(conf);
    //bindToGRPC("localhost:34015");
    return EXIT_SUCCESS;
}
