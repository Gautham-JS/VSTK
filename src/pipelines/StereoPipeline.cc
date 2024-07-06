#include "pipelines/StereoPipeline.hpp"
#include "io/Persistence.hpp"
#include "utils/GenericUtils.hpp"
#include "utils/DataUtils.hpp"

using namespace vstk;

StereoPipeline::StereoPipeline(VstkConfig conf) :
  conf(conf)
{}


void StereoPipeline::run() {
  INFOLOG("Starting Stereo Pipeline");
  const std::string rt_id = vstk::generate_random_string();
  DBGLOG(
        "\n=========================================================\nFeature Extraction Algorithm : %s\nDescriptor Compute Algorithm : %s\nFeature Matching Algorithm : %s\n\nRuntime ID : %s\n=========================================================\n", 
        vstk::enum_to_str(conf.get_feature_extraction_algo()), 
        vstk::enum_to_str(conf.get_descriptor_compute_algo()),
        vstk::enum_to_str(conf.get_match_algorithm()),
        rt_id
  );
  
  Timer t_main = get_timer("Stereo Pipeline");
  FeatureExtractor extractor(this->conf);
  FeatureMatcher matcher(this->conf);
  StereoTriangulate triangulator(this->conf);
  std::vector<ImageContextHolder> image_list;
  std::vector<FPSDataPt_t> fps_data;
  
  PersistenceConfig pconf;
  ProcMemoryPersistence memory_store(pconf);
  memory_store.initialize();



  // Grab the image file names
  vstk::DiskIO io;
  std::vector<std::string> l_files = io.list_directory(conf.get_stereo_src_1());
  std::vector<std::string> r_files = io.list_directory(conf.get_stereo_src_2());
  
  // Grab the genesis pair
  ImageContextHolder iml(l_files[0]);
  ImageContextHolder imr(r_files[0]);

  // TODO : Parallelize this
  extractor.run(iml);
  extractor.run(imr);
  MatchesHolder stereo_matches = matcher.run(iml, imr);
  
  DBGLOG("Initial left features : %d, Descriptors : %d", iml.get_features_holder().kps.size(), iml.get_features_holder().descriptors.size());
  DBGLOG("Initial right features : %d, Descriptors : %d", imr.get_features_holder().kps.size(), imr.get_features_holder().descriptors.size());

  image_list.reserve(l_files.size());
  image_list.push_back(iml);

  // TODO : Triangulation
  // Next we triangulate the genesis pair for
  // first reference 3D points and camera pose 
  // for initializing the Mapping and Localization models
  triangulator.run_sparse(iml, imr, stereo_matches);
  
  memory_store.set_reference_stereo_frame(rt_id, {iml, imr});
  
  // TODO : iteration currently limited to filesystem, once data loader 
  // design is final, need to incorporate it here and use it polymorphically. 
  size_t curr_idx = 1;
  size_t prev_idx = 0;
  size_t iter_lim = 1000000;        // hard stop on the iters.
  size_t iter = 0;                  // iter counter
  while(curr_idx < l_files.size()) {
    iter++;
    if(iter >= iter_lim) {
      ERRORLOG("Critical program failure, iteration limit exceeded for main loop.");
      exit(-1);
    }
    start_timer(t_main);
    INFOLOG("Loading image [ %ld / %ld ]", curr_idx, l_files.size());
    
    ImageContextHolder prev_im = memory_store.get_reference_stereo_frame(rt_id)->first;
    ImageContextHolder curr_im(l_files[curr_idx]);
    ImageContextHolder curr_im_r(r_files[curr_idx]);
    DBGLOG("PrevId : %s, CurrentId : %s", prev_im.get_image_id(), curr_im.get_image_id());
    if(prev_im.get_image().empty()) {
      WARNLOG("Image %s has empty data!", prev_im.get_image_id());
      prev_im.load_image_path(prev_im.get_image_id());
    }

    if(curr_im.get_image().empty()) {
      WARNLOG("Image %s has empty data!", curr_im.get_image_id());
      curr_im.load_image_path(l_files[curr_idx]);
    }
    extractor.run(curr_im);
    extractor.run(curr_im_r);

    MatchesHolder holder = matcher.run(curr_im, prev_im);
    stereo_matches = matcher.run(curr_im, curr_im_r);
    DBGLOG("Sequential Matcher detected %ld matches with new candidate image.", holder.good_matches.size());
    if(holder.good_matches.size() == 0) {
      WARNLOG("No matches detected in current and reference image, tracking lost.");
      curr_idx++;
      continue;
    }
    // Render the match display window, for debugging.
    matcher.display_match_overlap(curr_im, prev_im, holder);

    if(stereo_matches.good_matches.size() > 40) {
      triangulator.run_sparse(curr_im, curr_im_r, stereo_matches);
    }
    else {
      WARNLOG("Too few matches to triangulate points, skipping trinagulation phase");
    }

    if(holder.good_matches.size() > 40) {
      INFOLOG("Skipped current image, good quantity of matches.");
      curr_im.clear_image_data();
    }
    else {
      // matches below 40 but not zero, good point to sample new reference image
      INFOLOG("Inserting image %s as new reference image", curr_im.get_image_id());
      memory_store.set_reference_stereo_frame(rt_id, {curr_im, curr_im_r});
      prev_idx++;
    }
    end_timer(t_main);
    log_fps(t_main, stdout);
    fps_data.push_back(
      vstk::create_fps_data_pt(
        iml.get_image_id(), vstk::get_fps(t_main)
      )
    );

    // dump frame time data to json file every 50 frames to avoid disk IO bottleneck for every frame.
    if(curr_idx % 50) {
      vstk::dump_data_pts_to_file("framerate_vstk.json", fps_data);
    }
    curr_idx++;
  }
}

