#include "pipelines/StereoPipeline.hpp"

using namespace vstk;

StereoPipeline::StereoPipeline(DataLoader loader_left, DataLoader loader_right, VstkConfig conf) :
  l_loader(loader_left),
  r_loader(loader_right),
  conf(conf)
{}


void StereoLoader::run() {
  INFOLOG("Starting Stereo Pipeline");
  DBGLOG(
        "\n=========================================================\nFeature Extraction Algorithm : %s\nDescriptor Compute Algorithm : %s\nFeature Matching Algorithm : %s\n=========================================================\n", 
        vstk::enum_to_str(conf.get_feature_extraction_algo()), 
        vstk::enum_to_str(conf.get_descriptor_compute_algo()),
        vstk::enum_to_str(conf.get_match_algorithm())
  );
  
  Timer t_main = get_timer("Stereo Pipeline");
  FeatureExtractor extractor(this->conf);
  FeatureMatcher matcher(this->conf);
  
  // Grab the image file names
  DiskIO io;
  std::vector<std::string> l_files = io.list_directory(conf.get_stereo_src_1());
  std::vector<std::string> r_files = io.list_directory(conf.get_stereo_src_2());
  
  // Grab the genesis pair
  ImageContextHolder iml(l_files[0]);
  ImageContextHolder imr(r_files[0]);





  // Next we triangulate the genesis pair for
  // first reference 3D points and camera pose 
  // for initializing the Mapping and Localization models.
  


  

   



}

