#include "calib/CameraCalibration.hpp"
#include "config/CalibConfig.hpp"
#include "utils/Logger.hpp"

void print_usage_and_unalive(char *argv[]) {
    fprintf(stderr, "%s mode (stereo/mono) directory1_pattern (required) directory2_pattern (optional, used for stereo) \n[-s size:string -- checkerboard size represented as {cols}x{rows}]\n\
    [-o output directory:string -- path to create the calibration outputs]\n\
    [-v -- enable verbose mode]\n",
    argv[0]
    );
}

std::vector<std::string> split(std::string s, std::string delimiter) {
    size_t pos_start = 0, pos_end, delim_len = delimiter.length();
    std::string token;
    std::vector<std::string> res;

    while ((pos_end = s.find(delimiter, pos_start)) != std::string::npos) {
        token = s.substr (pos_start, pos_end - pos_start);
        pos_start = pos_end + delim_len;
        res.push_back (token);
    }

    res.push_back (s.substr (pos_start));
    return res;
}


vstk::CalibConfig build_config_from_args(int argc, char* argv[]) {
    vstk::CalibConfig conf;
    std::string mode_str;
    std::string checkerboard_dims;
    std::string config_file_path = "";
    char c;
    opterr = 0;

    while ((c = getopt (argc, argv, "o:s:v")) != -1) {
        switch (c) {
            case 's':
                checkerboard_dims = std::string(optarg);
                break;

            case 'v':
                vstk::Logger::get().enable_debug();
                break;

            case 'o':
                conf.out_dir = std::string(optarg);
                break;

            default:
                ERRORLOG("Unknwon option");
                abort ();
        }
    }

    if(conf.out_dir.empty()) {
        conf.out_dir = ".";
    }
    
    if(argc - optind < 2) {
        ERRORLOG("mode and/or directory not specified");
        print_usage_and_unalive(argv);
    }
    mode_str = std::string(argv[optind]);
    
    if(mode_str == "mono") {
        conf.mode = vstk::CALIB_MODE::MONO;
    }
    else if(mode_str == "stereo") {
        conf.mode = vstk::CALIB_MODE::STEREO;
    }
    else {
        ERRORLOG("Unsupported mode %s", mode_str);
        print_usage_and_unalive(argv);
    }

    conf.cam1.dir_pattern = argv[optind + 1];

    if(argc - optind == 3) {
        conf.cam2.dir_pattern = argv[optind + 2];
    }

    for (auto & ch: checkerboard_dims) ch = toupper(ch);
    std::vector<std::string> tokens = split(checkerboard_dims, "X");
    if(tokens.size() != 2) {
        ERRORLOG("Invalid syntax for checkerboard dimensions : %s", checkerboard_dims);
        print_usage_and_unalive(argv);
    }
    conf.ncols = atoi(tokens[0].c_str());
    conf.nrows = atoi(tokens[1].c_str());

    return conf;
}



int main(int argc, char *argv[]) {
    if (argc <= 2) {
        ERRORLOG("Calib Mode and File pattern of calibration images not specified");
        exit(1);
    }
    vstk::CalibConfig config = build_config_from_args(argc, argv);
    vstk::CameraCalibrator calibrator(std::make_pair(config.ncols, config.nrows));

    switch (config.mode) {
    case vstk::CALIB_MODE::MONO :
        calibrator.run(config.cam1);
        vstk::write_mono_config(config.cam1, config.out_dir + "/cam.yaml");
        break;
    case vstk::CALIB_MODE::STEREO :
        calibrator.run(config);
        vstk::write_stereo_config(config, config.out_dir + "/stereo_cam.yaml");
    default:
        break;
    }
    return 0;
}





