#include <stdio.h>
#include <string>


#ifndef __X3DS_IO_INTERFACE
#define __X3DS_IO_INTERFACE

#include "features/FeatureExtractor.hpp"
#include "config/Config.hpp"
#include "io/Persistence.hpp"

namespace vstk {

    class IOInterface {
        protected:
            vstk::PersistenceConfig config;

        public:
            explicit IOInterface(vstk::PersistenceConfig persistence_config) : config(persistence_config) {
            }
            
            // initialize the data stores
            virtual int initialize();

            // returns the next reference frame
            virtual ImageContextHolder get_next_frame();
            virtual StereoImageContextPair get_next_stereo_frame();

            // returns the previous reference frame from the appropriate data store. 
            ImageContextHolder get_reference_frame();
            StereoImageContextPair get_reference_stereo_frame();
    };
}

#endif