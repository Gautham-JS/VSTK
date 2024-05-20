#include "io/DiskIO.hpp"
#include "utils/Logger.hpp"


#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <filesystem>
#include <glob.h>

#define FS_DIR_EXISTS(FS_PATH) boost::filesystem::exists(FS_PATH)
#define FS_APPEND_PATHS(PARENT_DIR, DIR) ( \
    boost::format("%s/%s") \
    % PARENT_DIR \
    % DIR \
).str()

vstk::DiskIO::DiskIO(std::string base_directory) 
    : base_directory(base_directory), working_directory(get_working_directory())
{
//     this->create_base_dir();
}

vstk::DiskIO::DiskIO() {
    base_directory = "/tmp/vstk/data";
    working_directory = "/tmp/vstk/var";
}

vstk::DiskIO::DiskIO(std::string base_directory, std::string working_directory) 
    : base_directory(base_directory), working_directory(working_directory) {
        this->create_base_dir();
}


std::string vstk::DiskIO::append_paths(std::string parent_dir, std::string dir) {
    boost::format fmt = boost::format("%s/%s") % parent_dir % dir;
    return fmt.str();
}

std::string vstk::DiskIO::get_working_directory() {
    const char *working_dir = getenv("vstk_FS_ROOT");
    if(working_dir == nullptr) {
        working_dir = "/tmp/vstk_fs/";
    }
    std::string vstk_fs = std::string(working_dir); 
    // if ( !FS_DIR_EXISTS(vstk_fs)) {
    //     DBGLOG("Creating working directory %s", vstk_fs.c_str());
    //     if(!boost::filesystem::create_directory(vstk_fs)) {
    //         ERRORLOG("Failed to create a filesystem directory at %s", vstk_fs.c_str());
    //         throw std::filesystem::filesystem_error("ERR_WORKING_DIR_CREATE", std::error_code());
    //     }
    // }
    return vstk_fs;
}

void vstk::DiskIO::create_base_dir() {
    this->dir = FS_APPEND_PATHS(base_directory, working_directory);
    if(!FS_DIR_EXISTS(this->dir)) {
        DBGLOG("Creating base directory %s/%s", base_directory.c_str() ,working_directory.c_str());
        this->dir = FS_APPEND_PATHS(base_directory, working_directory);
        if(!boost::filesystem::create_directory(this->dir)) {
            ERRORLOG("Failed to create base directory for operation");
            throw std::filesystem::filesystem_error("ERR_BASE_DIR_CREATE", std::error_code());
        }
    }
}

void vstk::DiskIO::write(std::string filename, cv::Mat image) {
    DBGLOG("Writing image data to path : %s", filename.c_str());
    cv::imwrite(FS_APPEND_PATHS(this->dir, filename),  image);
}

void vstk::DiskIO::write(std::string filename, std::string data) {
    DBGLOG("Writing data to path : %s", filename.c_str());
    
}

std::string vstk::DiskIO::get_file_name(std::string full_path) {
    boost::filesystem::path path(full_path);
    return path.stem().string();
}


std::string vstk::DiskIO::get_parent_directory(std::string directory) {
    boost::filesystem::path path(path);
    return path.parent_path().string();
}

std::vector<std::string> vstk::DiskIO::list_directory(std::string directory_pattern) {
    glob_t glob_result;
    memset(&glob_result, 0, sizeof(glob_result));
    int return_value = glob(directory_pattern.c_str(), GLOB_TILDE, NULL, &glob_result);
    if(return_value != 0) {
        globfree(&glob_result);
        ERRORLOG("Glob Failed with return code %d", return_value)
        throw std::filesystem::filesystem_error("ERR_GLOB_FAILED", std::error_code());
    }

    std::vector<std::string> filenames;
    for(size_t i = 0; i < glob_result.gl_pathc; ++i) {
        std::string file = std::string(glob_result.gl_pathv[i]);
        filenames.push_back(file);
    }

    // cleanup
    globfree(&glob_result);
    return filenames;
}
