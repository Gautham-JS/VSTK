#ifndef __VSTK_DATA_LOADER_H
#define __VSTK_DATA_LOADER_H

#include <stdio.h>
#include <string>

#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"

#include "io/IOInterface.hpp"

namespace vstk {
    class FilesystemIO : public IOInterface {
        private:
            VstkConfig conf;
        public:
            explicit FilesystemIO(vstk::PersistenceConfig config, vstk::VstkConfig vstk_config) : IOInterface(config), conf(vstk_config) {}

            ImageContextHolder get_next_frame() override;
            StereoImageContextPair get_next_stereo_frame() override;

    };
}

#endif