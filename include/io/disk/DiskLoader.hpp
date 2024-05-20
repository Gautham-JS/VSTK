#ifndef __VSTK_DISK_LOADER_H
#define __VSTK_DISK_LOADER_H

#include <stdio.h>
#include <vector>
#include <string>

#include "io/DiskIO.hpp"
#include "io/DataLoader.hpp"

namespace vstk {

    class DiskLoader : public DataLoader {
        private:
            std::vector<std::string> files;
            uint idx = 0;

        public:
            explicit DiskLoader(std::string directory_pattern);
            DiskLoader();
            
            int load_source(std::string source);
            int iterate_next_image(cv::Mat &image);
            bool is_iteration_complete();
            std::vector<std::string> get_all_files();
    };
}

#endif
