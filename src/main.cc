#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>

#include "opencv2/opencv.hpp"

#include "core/Runtime.hpp"
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
    vstk::set_async(true);
    vstk::describe_config(conf);
    RuntimeManager rt_manager;
    VstkConfig conf2(conf);

    vstk::set_headless(false);

    std::string id1 = rt_manager.start_runtime(conf);
    std::this_thread::sleep_for(std::chrono::seconds(5));
    std::string id2 = rt_manager.start_runtime(conf2);
    std::this_thread::sleep_for(std::chrono::seconds(5));
    std::string id3 = rt_manager.start_runtime(conf);
    std::this_thread::sleep_for(std::chrono::seconds(5));
    std::string id4 = rt_manager.start_runtime(conf2);

    this_thread::sleep_for(std::chrono::seconds(30));
    if(!rt_manager.is_running(id1)) {
        ERRORLOG("Running RT1 shown as not running by manager");
        exit(-1);
    }
    if(!rt_manager.is_running(id2)) {
        ERRORLOG("Running RT shown as not running by manager");
        exit(-1);
    }
    INFOLOG("Runtime %s and %s running as expected, gracefully stopping both", id1, id2);
    rt_manager.stop_runtime(id1);
    rt_manager.stop_runtime(id2);
    std::this_thread::sleep_for(std::chrono::seconds(5));
    rt_manager.stop_runtime(id3);
    rt_manager.stop_runtime(id4);
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
