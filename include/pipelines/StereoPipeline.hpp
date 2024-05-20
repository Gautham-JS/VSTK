#include <stdio.h>
#include <vector>


#ifndef __STEREO_PIPELINE_H
#define __STEREO_PIPELINE_H

#include "utils/Logger.hpp"
#include "features/FeatureExtractor.hpp"
#include "features/FeatureMatcher.hpp"
#include "config/Config.hpp"

namespace vstk {
  class StereoPipeline {
    private:
      VstkConfig conf;
    public:
      explicit StereoPipeline(VstkConfig conf);
      void run();
      int run_asyc();
  };
}

#endif