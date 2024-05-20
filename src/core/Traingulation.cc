#include "core/Triangulation.hpp"

using namespace vstk;

StereoTriangulate::StereoTriangulate(VstkConfig conf) : conf(conf) {}

CamView StereoTriangulate::run_sparse(ImageContextHolder l_im, ImageContextHolder r_im) {
  INFOLOG("Running sparse stereo triangulation on image pair [%s] & [%s]",l_im.get_image_id(), r_im.get_image_id());
  CamView camera_view;

}
