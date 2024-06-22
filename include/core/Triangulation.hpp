#ifndef __VSTK_TRAINGULATION_H
#define __VSTK_TRIANGULATION_H

#include <stdio.h>
#include <string>
#include <vector>

#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/calib3d.hpp"

#include "config/Config.hpp"
#include "features/FeatureExtractor.hpp"
#include "features/FeatureMatcher.hpp"
#include "utils/Logger.hpp"


namespace vstk {
  // Represents the camera view parameters
  // generated using the traingulation process,
  // 3D Points are in the camera frame,
  // needs transformation for world frame representation (TBD)

  class StereoTriangulate {
    private:
      VstkConfig conf;
    public:
      explicit StereoTriangulate(VstkConfig conf);
      CamView run_sparse(ImageContextHolder l_im, ImageContextHolder r_im, MatchesHolder match_holder);
  };
}

#endif
