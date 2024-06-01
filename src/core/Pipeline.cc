#include "core/Pipeline.hpp"
#include "utils/TimerUtils.hpp"
#include "core/Triangulation.hpp"

using namespace vstk;

const std::string working_dir = "/home/gjs/software/vstk/data/";

StereoPipeline::StereoPipeline() {}

void StereoPipeline::start(VstkConfig conf) {
    INFOLOG("Starting VSTK runtime in blocking mode with stereo configuration");
    DBGLOG(
        "\n=========================================================\nFeature Extraction Algorithm : %s\nDescriptor Compute Algorithm : %s\nFeature Matching Algorithm : %s\n=========================================================\n", 
        vstk::enum_to_str(conf.get_feature_extraction_algo()), 
        vstk::enum_to_str(conf.get_descriptor_compute_algo()),
        vstk::enum_to_str(conf.get_match_algorithm())
    );
    DiskIO left_io(working_dir, "matches");
    DiskIO right_io(working_dir, "matches");
    FeatureExtractor extractor(conf);
    FeatureMatcher matcher(conf);
    StereoTriangulate triangulator(conf);


    std::vector<std::string> left_files = left_io.list_directory(conf.get_stereo_src_1());
    std::vector<std::string> right_files = right_io.list_directory(conf.get_stereo_src_2());
    std::vector<ImageContextHolder> ref_image_list;
    int cur_ptr = 1;
    int prev_ptr = 0;


    if(left_files.size() != right_files.size()) {
        ERRORLOG("Filesystem setup error : Left and Right stereo image file sets have different sizes");
        exit(-1);
    }

    Timer t_main = get_timer("Main");

    //grab images from left and right sources
    ImageContextHolder image_left(left_files[0]);
    ImageContextHolder image_right(right_files[0]);
    extractor.run(image_left);
    extractor.run(image_right);
    MatchesHolder stereo_matches = matcher.run(image_left, image_right);
    
    //triangulate reference points
    INFOLOG("Running triangulator...");
    triangulator.run_sparse(image_left, image_right, stereo_matches);

    ref_image_list.push_back(image_left);

    while(cur_ptr < left_files.size()) {
        start_timer(t_main);
        INFOLOG("Loading image [ %d / %d ]", cur_ptr, left_files.size() - 1);
        ImageContextHolder prev_image = ref_image_list[prev_ptr];
        ImageContextHolder curr_image (left_files[cur_ptr]);
        extractor.run(curr_image);
        MatchesHolder match_holder = matcher.run(curr_image, prev_image);
        DBGLOG("Detected %d features.", match_holder.good_matches.size());

        if(match_holder.good_matches.size() == 0) {
            WARNLOG("No good quality matches, rejecting image pair");
            cur_ptr++;
            continue;
        }
        matcher.display_match_overlap(curr_image, prev_image, match_holder);
        if(match_holder.good_matches.size() > 40) {
            curr_image.clear_image_data();
            cur_ptr++;
            continue;
        }
        else {
            INFOLOG("[INSERT] Diminishing match quantity, inserting reference image & clearing image %d from memory", prev_ptr);
            // right here, we triangulate points for current ptr which will be prev_image in next iter.
            ref_image_list[prev_ptr].clear_image_data();
            ref_image_list.emplace_back(curr_image);
            prev_ptr++;
        }
        end_timer(t_main);
        log_fps(t_main, stdout);
        cur_ptr++;
    }

}

