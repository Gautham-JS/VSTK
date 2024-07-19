#include "io/IOInterface.hpp"

#include <algorithm>


using namespace vstk;



int FileIO::init_stereo() {
    DBGLOG("Initializing File IO in STEREO mode");
    std::string left_src = this->config.get_stereo_src_1();
    std::string right_src = this->config.get_stereo_src_2();

    if(left_src.empty() || left_src.size() == 0) {
        ERRORLOG("Stereo-Left images directory config could not be read");
        return enum_as_integer(IO_ERROR_STATES::CFG_READ_ERR);
    }
    if(right_src.empty() || right_src.size() == 0) {
        ERRORLOG("Stereo-Right images directory config could not be read");
        return enum_as_integer(IO_ERROR_STATES::CFG_READ_ERR);
    }
    
    this->files_left = io.list_directory(this->config.get_stereo_src_1());
    this->files_right = io.list_directory(this->config.get_stereo_src_2());

    if(this->files_left.size() == 0 || this->files_right.size() == 0) {
        ERRORLOG("File IO Error, Directory list failed. Stereo left or right directory sets empty!");
        return enum_as_integer(IO_ERROR_STATES::CFG_VALIDATION_ERR);
    }

    this->fs_ilimit = std::min(this->files_left.size(), this->files_right.size());

    return enum_as_integer(IO_ERROR_STATES::OK);
}

int FileIO::initialize() {
    DBGLOG("Initializing Filesystem data store");
    int rc = EXIT_FAILURE;

    switch(this->config.get_slam_type()){
        case SLAMType::STEREO :
            rc = init_stereo();
            break;
        default:
            ERRORLOG("SLAM Type unsupported");
            break;
    }

    
    if(rc == EXIT_SUCCESS)  { DBGLOG("Completed Filesystem data store initialization"); }
    else                    { ERRORLOG("Failed to initialize File IO layer"); }

    return rc;
}


StereoImageContextPair FileIO::get_next_stereo_frame() {
    std::unique_lock<std::shared_mutex>(io_mtx);
    if(this->fs_iptr >= this->fs_ilimit) {
        ERRORLOG("File iteration complete, resetting file-iteration ptr");
        this->fs_iptr = 0;
    }
    ImageContextHolder left = ImageContextHolder(this->files_left[fs_iptr]);
    ImageContextHolder right = ImageContextHolder(this->files_right[fs_iptr]);
    this->fs_iptr++;
    return {left, right};
}

ImageContextHolder FileIO::get_next_frame() {
    if(this->fs_iptr >= this->fs_ilimit) {
        ERRORLOG("File iteration complete, resetting file-iteration ptr");
        this->fs_iptr = 0;
    }
    return ImageContextHolder(this->files_mono[fs_iptr++]);
}

bool FileIO::is_io_active() {
    return (this->fs_iptr < this->fs_ilimit);
}