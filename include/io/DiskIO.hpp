#include "io/IOInterface.hpp"

#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>


#ifndef __VSTK_DISK_IO_H
#define __VSTK_DISK_IO_H

namespace vstk {
    /*
    DiskIO : Wrapper class for all Disk input/output operations.
    -> Accepts a file path as an constructor that is used for writing/reading any program data that needs to be dumped to disk.
    -> Provides wrappers for reading and writing objects into disk.
    Future enhancements : 
        -> can be a standalone thread that handles disk IO, objects can be stored in memory for quick access and then written to disk if read has not been performed in a while (some kinda timer in this thread)

    */
   typedef struct {
        std::string name;
        bool is_directory;
   } FSDescriptor;

    class DiskIO {
        private:
            std::string base_directory;
            std::string working_directory;
            std::string dir;

            std::string append_paths(std::string parent_dir, std::string dir);
            void create_base_dir();
            std::string get_working_directory();

        public :
            explicit DiskIO();
            explicit DiskIO(std::string base_directory);
            explicit DiskIO(std::string base_directory, std::string working_dirtectory);

            // functions for writing / reading files as raw string objcts.
            void write(std::string filename, std::string data);
            void write(std::string filename, cv::Mat image);
            void create_directory(std::string directory);
            std::string read(std::string filename);
            std::string get_file_name(std::string full_path);
            std::string get_parent_directory(std::string directory);

            std::vector<std::string> list_directory(std::string directory_pattern);            


            // functions for read/write files as chunks. 
            std::string read_chunk(std::string filename, uint32_t chunk_number);
            void write_chunk(std::string filename, std::string data, uint32_t chunk_number);
            void join_chunks(std::string filename);
    };
}

#endif