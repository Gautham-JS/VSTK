#include <stdio.h>
#include "opencv2/opencv.hpp"
#include "grpcpp/grpcpp.h"


#include "features/FeatureExtractor.hpp"
#include "features/FeatureMatcher.hpp"
#include "protoimpl/ImageSvc.hpp"
#include "utils/Logger.hpp"




using namespace std; 

void runMatcher() {
    INFOLOG("Running feature matcher");

    std::unique_ptr<cv::Mat> im1 = std::make_unique <cv::Mat> (cv::imread("/home/gjs/Downloads/rm1.jpg", cv::IMREAD_GRAYSCALE));
    std::unique_ptr<cv::Mat> im2 = std::make_unique <cv::Mat> (cv::imread("/home/gjs/Downloads/rm2.jpg", cv::IMREAD_GRAYSCALE));
    
    x3ds::FeatureExtractor extractor1(std::move(im1));
    x3ds::FeatureExtractor extractor2(std::move(im2));
    std::unique_ptr<x3ds::FeatureExtractor> eptr1 = std::make_unique<x3ds::FeatureExtractor>(extractor1);
    std::unique_ptr<x3ds::FeatureExtractor> eptr2 = std::make_unique<x3ds::FeatureExtractor>(extractor2);

    eptr1.get()->run_sift();
    eptr2.get()->run_sift();

    x3ds::FeatureMatcher matcher(std::move(eptr1), std::move(eptr2));
    matcher.run();
    matcher.display_matches();

}

void bindToGRPC(std::string addr) {

    INFOLOG("Attempting to bind using grpc to address %s", addr.c_str());
    x3ds::ImageSvc image_service;
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
    bindToGRPC("localhost:34015");
    return 0;
}