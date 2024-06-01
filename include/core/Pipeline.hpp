#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

#include <unordered_map>

#include "features/FeatureExtractor.hpp"
#include "features/FeatureMatcher.hpp"
#include "config/Config.hpp"

#include "utils/Logger.hpp"
#include "utils/CvUtils.hpp"
#include "io/DiskIO.hpp"


#ifndef __VSTK_CORE_STEREO_PIPELINE_H
#define __VSTK_CORE_STEREO_PIPELINE_H

namespace vstk {

    class IPipeline {
        public:
            virtual void start(VstkConfig config);
            virtual std::string start_async(VstkConfig config);
            virtual int stop_async(std::string rt_id);
            virtual bool is_running(std::string rt_id); 
    };

    class StereoPipeline : public IPipeline {
        private:
        public:
            StereoPipeline();
            void start(VstkConfig config);
            std::string start_async();
            int stop_async(std::string rt_id);
            bool is_running(std::string rt_id);
    };

    class PipelineManager {
        private:
            std::unordered_map<std::string, IPipeline*> pipelines;
        public:
            std::string start_pipeline(VstkConfig config, IPipeline* &pipeline);
            bool is_running(std::string rt_id);
            int stop_pipeline(std::string rt_id);
    };
}

#endif