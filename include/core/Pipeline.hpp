#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

#include <unordered_map>

#include "features/FeatureExtractor.hpp"
#include "features/FeatureMatcher.hpp"
#include "config/Config.hpp"

#include "utils/Logger.hpp"
#include "utils/CvUtils.hpp"
#include "io/DiskIO.hpp"
#include "io/IOInterface.hpp"
#include "io/Persistence.hpp"
#include "utils/GenericUtils.hpp"


#ifndef __VSTK_CORE_STEREO_PIPELINE_H
#define __VSTK_CORE_STEREO_PIPELINE_H

namespace vstk {

    class IPipeline {
        protected:
            std::shared_ptr<IOInterface> io_layer;
            std::shared_ptr<IPersistentDataStore> persistence_layer;
            vstk::VstkConfig conf;
            std::string rt_id;
        
        private:
            inline IPersistentDataStore* build_persistence_layer(VstkConfig conf) {
                return new ProcMemoryPersistence(conf);
            }

            inline IOInterface* build_io_layer(VstkConfig conf) {
                return new FileIO(conf);
            }

        public:  
            explicit IPipeline(VstkConfig conf) : 
                conf(conf),
                rt_id(generate_random_string(16)),
                io_layer(this->build_io_layer(conf)),
                persistence_layer(this->build_persistence_layer(conf))
            {};

            explicit IPipeline(VstkConfig conf, std::string rt_id) : 
                conf(conf),
                rt_id(rt_id),
                io_layer(this->build_io_layer(conf)),
                persistence_layer(this->build_persistence_layer(conf))
            {};
            virtual void initialize() = 0;
            virtual void start() = 0;
    };

    class StereoPipeline : public IPipeline {
        public:
            StereoPipeline(VstkConfig conf);
            StereoPipeline(VstkConfig conf, std::string rt_id);
            
            void initialize() override;
            void start() override;
    };

}

#endif