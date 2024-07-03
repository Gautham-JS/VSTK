#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>

#include "opencv2/opencv.hpp"

#include "pipelines/StereoPipeline.hpp"
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
    for(size_t i=0; i<files.size(); i++) {
        ImageContextHolder ictx(files[i]);
        extractor.run(ictx);
        vstk::BinaryDataStream *data = serializer.serialize(ictx);
        cerr << i << " : " <<data->data << endl;
    }
}

void run_locally(VstkConfig conf) {
    StereoPipeline pipeline(conf);
    pipeline.run();
}

void bindToGRPC(const std::string &addr) {
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


int main(int argc, char** argv ) {
    vstk::VstkConfig conf = vstk::build_config_from_args(argc, argv);
    run_locally(conf);
    //bindToGRPC("localhost:34015");
    return EXIT_SUCCESS;
}
