#include <stdio.h>
#include <string>
#include <queue>

#ifndef __VSTK_IO_INTERFACE_H_
#define __VSTK_IO_INTERFACE_H_

#include "features/FeatureExtractor.hpp"
#include "config/Config.hpp"
#include "io/Persistence.hpp"
#include "io/DiskIO.hpp"
#include "utils/Logger.hpp"

namespace vstk {

    /**
     * High level interface for Application Data Layer.
     * 
     * Interfaces Unstructured data in data stores to structured image streams.
     * 
     * 
     * Responsibilities : 
     *  -> Fetch Input Image.
     *  -> Setup persistence layer for reference frames and other non volatile data.
     *  -> Write data types to data stores (Pointclouds, 2D point sets, frame positions etc for re-use).
     */

    class IOInterface {
        protected:
            vstk::VstkConfig config;
        public:
            explicit IOInterface(vstk::VstkConfig config) : config(config) {
            }
            
            // initialize the data stores
            virtual int initialize() = 0;

            // returns the next reference frame
            virtual StereoImageContextPair get_next_stereo_frame() = 0;
            virtual ImageContextHolder get_next_frame() = 0;

            virtual bool is_io_active() = 0;
    };

    class FileIO final : public IOInterface {
        private:
            size_t fs_iptr = 0;                     // Filesystem read-iteration pointer                  
            size_t fs_ilimit = 0;                   // Filesystem iteration limit (min number of files in l/r stores)

            DiskIO io;                              // IO handler
            std::vector<std::string> files_mono;    // monocular
            std::vector<std::string> files_left;    // stereo left 
            std::vector<std::string> files_right;   // stereo right

            int init_stereo();
            int init_mono();

        public:
            explicit FileIO(VstkConfig conf) : IOInterface(conf) {}
            int initialize() override;

            bool is_io_active() override;
            StereoImageContextPair get_next_stereo_frame() override;
            ImageContextHolder get_next_frame() override;
    };

    class RosIO final : public IOInterface {
        private:

    };

}

#endif