#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>

#include "opencv2/opencv.hpp"

#include "core/Pipeline.hpp"
#include "features/FeatureExtractor.hpp"
#include "features/FeatureMatcher.hpp"
#include "utils/Logger.hpp"
#include "config/Config.hpp"
#include "io/DiskIO.hpp"
#include "io/Serializer.hpp"
#include "utils/TimerUtils.hpp"



using namespace std; 
using namespace vstk;

const std::string working_dir = "/home/gjs/software/vstk/data/";
const std::string im_pattens = "/media/gjs/Windows-SSD/Data/TUM/dataset-outdoors3_512_16/mav0/cam0/data/*.png";

void run_locally(VstkConfig conf) {
    StereoPipeline pipeline(conf, "test_rt");
    pipeline.initialize();
    pipeline.start();
}

void print_usage_and_unalive(char *prog_name) {
    fprintf(stderr, "%s data_source <args>\n", prog_name);
    exit(EXIT_FAILURE);
}


int main(int argc, char** argv ) {
    vstk::VstkConfig conf = vstk::build_config_from_args(argc, argv);
    run_locally(conf);
    return EXIT_SUCCESS;
}
