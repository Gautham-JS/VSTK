#include "io/disk/DiskLoader.hpp"
#include "utils/Logger.hpp"

using namespace vstk;


DiskLoader::DiskLoader() {}

DiskLoader::DiskLoader(std::string directory_pattern) {
    this->load_source(directory_pattern);
}

int DiskLoader::load_source(std::string source) {
    INFOLOG("Loading image files from disk using pattern : %s", source);
    DiskIO dio;
    this->files = dio.list_directory(source);
    DBGLOG("Found %ld file references.", this->files.size());
    if(files.size() != 0) return 0;
    else return 1; 
}

int DiskLoader::iterate_next_image(cv::Mat &image) {
    DBGLOG("Loading image [%ld/%ld] from disk", this->idx, this->files.size());
    int rc = 1;
    if(this->idx >= this->files.size()) return rc;

    cv::Mat m = cv::imread(this->files[this->idx]);
    
    if(m.empty()) return rc;
    image(m);
    this->idx++;
    return 0;
}

bool DiskLoader::is_iteration_complete() {
    return (this->idx >= this->files.size());
}

  




