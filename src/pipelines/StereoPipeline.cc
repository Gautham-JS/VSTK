#include "pipelines/StereoPipeline.hpp"

#include "io/IOInterface.hpp"
#include "io/Persistence.hpp"
#include "utils/DataUtils.hpp"
#include "utils/GenericUtils.hpp"

using namespace vstk;

StereoPipeline::StereoPipeline(VstkConfig conf) : conf(conf) {}

void StereoPipeline::run() {
    INFOLOG("Starting Stereo Pipeline");
    const std::string rt_id = vstk::generate_random_string();
    DBGLOG(
        "\n=========================================================\nFeature Extraction Algorithm : %s\nDescriptor Compute Algorithm : %s\nFeature Matching Algorithm : %s\n\nRuntime ID : %s\n=========================================================\n",
        vstk::enum_to_str(conf.get_feature_extraction_algo()),
        vstk::enum_to_str(conf.get_descriptor_compute_algo()),
        vstk::enum_to_str(conf.get_match_algorithm()),
        rt_id);

    Timer t_main = get_timer("Stereo Pipeline");
    FeatureExtractor extractor(this->conf);
    FeatureMatcher matcher(this->conf);
    StereoTriangulate triangulator(this->conf);
    std::vector<ImageContextHolder> image_list;
    std::vector<FPSDataPt_t> fps_data;

    PersistenceConfig pconf;
    ProcMemoryPersistence memory_store(pconf);
    FileIO fio = FileIO(conf);

    fio.initialize();
    memory_store.initialize();

    StereoImageContextPair stereo_pair = fio.get_next_stereo_frame();

    // Grab the genesis pair
    ImageContextHolder iml(stereo_pair.first);
    ImageContextHolder imr(stereo_pair.second);

    // TODO : Parallelize this
    extractor.run(iml);
    extractor.run(imr);
    MatchesHolder stereo_matches = matcher.run(iml, imr);

    DBGLOG("Initial left features : %d, Descriptors : %d", iml.get_features_holder().kps.size(), iml.get_features_holder().descriptors.size());
    DBGLOG("Initial right features : %d, Descriptors : %d", imr.get_features_holder().kps.size(), imr.get_features_holder().descriptors.size());

    // TODO : Triangulation
    // Next we triangulate the genesis pair for
    // first reference 3D points and camera pose
    // for initializing the Mapping and Localization models
    triangulator.run_sparse(iml, imr, stereo_matches);

    memory_store.set_reference_stereo_frame(rt_id, {iml, imr});

    // TODO : iteration currently limited to filesystem, once data loader
    // design is final, need to incorporate it here and use it polymorphically.
    int curr_idx = 0;
    while (fio.is_io_active()) {
        start_timer(t_main);

        stereo_pair = fio.get_next_stereo_frame();
        ImageContextHolder prev_im = memory_store.get_reference_stereo_frame(rt_id)->first;
        ImageContextHolder curr_im(stereo_pair.first);
        ImageContextHolder curr_im_r(stereo_pair.second);
        DBGLOG("PrevId : %s, CurrentId : %s", prev_im.get_image_id(), curr_im.get_image_id());
        if (prev_im.get_image().empty()) {
            WARNLOG("Image %s has empty data!", prev_im.get_image_id());
            break;
        }

        if (curr_im.get_image().empty()) {
            WARNLOG("Image %s has empty data!", curr_im.get_image_id());
            break;
        }

        extractor.run(curr_im);
        extractor.run(curr_im_r);

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
            stereo_matches = matcher.run(curr_im, curr_im_r);
            if (stereo_matches.good_matches.size() > 40) {
                triangulator.run_sparse(curr_im, curr_im_r, stereo_matches);
            } else {
                WARNLOG("Too few matches to triangulate points, skipping trinagulation phase");
            }
            INFOLOG("Inserting image %s as new reference image", curr_im.get_image_id());
            memory_store.set_reference_stereo_frame(rt_id, {curr_im, curr_im_r});
        }
        end_timer(t_main);
        log_fps(t_main, stdout);
        fps_data.push_back(
            vstk::create_fps_data_pt(
                iml.get_image_id(), vstk::get_fps(t_main)));

        // dump frame time data to json file every 50 frames to avoid disk IO bottleneck for every frame.
        if (curr_idx % 50 == 0) {
            vstk::dump_data_pts_to_file("framerate_vstk.json", fps_data);
        }
        curr_idx++;
    }
}
