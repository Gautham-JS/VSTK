#include "core/Pipeline.hpp"

#include "core/Triangulation.hpp"
#include "utils/DataUtils.hpp"
#include "utils/GenericUtils.hpp"
#include "utils/TimerUtils.hpp"

using namespace vstk;

const std::string working_dir = "/home/gjs/software/vstk/data/";

StereoPipeline::StereoPipeline(vstk::VstkConfig conf) : 
    IPipeline(conf) 
{}
StereoPipeline::StereoPipeline(vstk::VstkConfig conf, std::string rt_id) : 
    IPipeline(conf, rt_id)
{}

void StereoPipeline::initialize() {
    INFOLOG("Initializing Stereo pipeline");
    if(this->rt_id.empty()) {
        INFOLOG("Stereo pipeline does not have an associated Runtime ID, generating new rt_id");
        this->rt_id = vstk::generate_random_string(16);
        DBGLOG("Built new random rt_id as %s", this->rt_id);
    }
    if (this->io_layer->initialize() != vstk::enum_as_integer(IO_ERROR_STATES::OK)) {
        throw std::runtime_error("Failed to initialize IO layer");
    }
    // TODO : Add return code for this initialization as its a critical operation
    // for the runtime
    this->persistence_layer->initialize();
    INFOLOG("Completed stereo pipeline initialization");
}

void StereoPipeline::start() {
    INFOLOG("Starting Stereo Pipeline");
    DBGLOG("\n\n");
    DBGLOG("\t--> Runtime ID :: %s\n", this->rt_id);
    DBGLOG("\t--> Feature Extraction Algorithm :: %s\n", vstk::enum_to_str(conf.get_feature_extraction_algo()));
    DBGLOG("\t--> Descriptor Compute Algorithm :: %s\n", vstk::enum_to_str(conf.get_descriptor_compute_algo()));
    DBGLOG("\t--> Point Matching Algorithm :: %s\n", vstk::enum_to_str(conf.get_match_algorithm()));
    DBGLOG("\n");

    Timer t_main = get_timer("Stereo Pipeline");
    FeatureExtractor extractor(this->conf);
    FeatureMatcher matcher(this->conf);
    StereoTriangulate triangulator(this->conf);
    std::vector<ImageContextHolder> image_list;
    std::vector<FPSDataPt_t> fps_data;

    StereoImageContextPair stereo_pair = this->io_layer->get_next_stereo_frame();

    // Grab the genesis pair
    ImageContextHolder iml(stereo_pair.first);
    ImageContextHolder imr(stereo_pair.second);

    // TODO : Parallelize this
    extractor.run(iml);
    extractor.run(imr);
    MatchesHolder stereo_matches = matcher.run(iml, imr);

    DBGLOG("Initial left features : %d, Descriptors : %d",
           iml.get_features_holder().kps.size(),
           iml.get_features_holder().descriptors.size()
    );
    DBGLOG("Initial right features : %d, Descriptors : %d",
           imr.get_features_holder().kps.size(),
           imr.get_features_holder().descriptors.size()
    );

    // TODO : Triangulation
    // Next we triangulate the genesis pair for
    // first reference 3D points and camera pose
    // for initializing the Mapping and Localization models
    triangulator.run_sparse(iml, imr, stereo_matches);

    this->persistence_layer->set_reference_stereo_frame(rt_id, {iml, imr});

    // TODO : iteration currently limited to filesystem, once data loader
    // design is final, need to incorporate it here and use it polymorphically.
    int curr_idx = 0;
    while (this->io_layer->is_io_active()) {
        start_timer(t_main);

        stereo_pair = this->io_layer->get_next_stereo_frame();
        ImageContextHolder prev_im = this->persistence_layer->get_reference_stereo_frame(rt_id)->first;
        ImageContextHolder curr_im(stereo_pair.first);
        ImageContextHolder curr_im_r(stereo_pair.second);

        DBGLOG("PrevId : %s, CurrentId : %s", prev_im.get_image_id(), curr_im.get_image_id());

        extractor.run(curr_im);

        MatchesHolder holder = matcher.run(curr_im, prev_im);
        DBGLOG("Sequential Matcher detected %ld matches with new candidate image.", holder.good_matches.size());
        if (holder.good_matches.size() == 0) {
            WARNLOG("No matches detected in current and reference image, tracking lost.");
            continue;
        }
        // Render the match display window, for debugging.
        matcher.display_match_overlap(curr_im, prev_im, holder);

        if (holder.good_matches.size() > 40) {
            INFOLOG("Skipped current image, good quantity of matches.");
            curr_im.clear_image_data();
        } else {
            // matches below 40 but not zero, good point to sample new reference image
            extractor.run(curr_im_r);
            stereo_matches = matcher.run(curr_im, curr_im_r);
            if (stereo_matches.good_matches.size() > 40) {
                triangulator.run_sparse(curr_im, curr_im_r, stereo_matches);
            } else {
                WARNLOG("Too few matches to triangulate points, skipping trinagulation phase");
            }
            INFOLOG("Inserting image %s as new reference image", curr_im.get_image_id());
            this->persistence_layer->set_reference_stereo_frame(rt_id, {curr_im, curr_im_r});
        }
        end_timer(t_main);
        log_fps(t_main, stdout);
        fps_data.push_back(
            vstk::create_fps_data_pt(iml.get_image_id(), vstk::get_fps(t_main))
        );

        // dump frame time data to json file every 50 frames to avoid disk IO
        // bottleneck for every frame.
        if (curr_idx % 50 == 0) {
            vstk::dump_data_pts_to_file("framerate_vstk.json", fps_data);
        }
        curr_idx++;
    }
}
