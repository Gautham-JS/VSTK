#include "features/FeatureExtractor.hpp"
#include "utils/Logger.hpp"
#include "utils/CvUtils.hpp"

#include <functional>
#include <random>
#include <sstream>

using namespace vstk;

static std::random_device               rd;
static std::mt19937                     gen(rd());
static std::uniform_int_distribution<>  dis(0, 15);
static std::uniform_int_distribution<>  dis2(8, 11);

std::string generate_uuid_v4() {
    std::stringstream ss;
    int i;
    ss << std::hex;
    for (i = 0; i < 8; i++) {
        ss << dis(gen);
    }
    ss << "-";
    for (i = 0; i < 4; i++) {
        ss << dis(gen);
    }
    ss << "-4";
    for (i = 0; i < 3; i++) {
        ss << dis(gen);
    }
    ss << "-";
    ss << dis2(gen);
    for (i = 0; i < 3; i++) {
        ss << dis(gen);
    }
    ss << "-";
    for (i = 0; i < 12; i++) {
        ss << dis(gen);
    };
    return ss.str();
}

ImageContextHolder::ImageContextHolder(std::string image_path) {
    load_image_path(image_path);
}

ImageContextHolder::ImageContextHolder(unsigned char *image_data, uint32_t image_width, uint32_t image_length) {
    load_image_data(image_data, image_length, image_width);
}

void ImageContextHolder::load_image_data(unsigned char *image_data, uint32_t image_length, uint32_t image_width) {
    this->image_width = image_width;
    this->image_length = image_length;
    this->image_id = generate_uuid_v4();
    this->image_data = cv::Mat(image_width, image_length, CV_8UC1, image_data);
}

void ImageContextHolder::load_image_path(std::string image_path) {
    this->image_data = cv::imread(image_path, cv::IMREAD_COLOR);
    this->image_id = image_path;
    this->image_length = this->image_data.cols;
    this->image_width = this->image_data.rows;
}

ImageContextHolder::ImageContextHolder() {

}

FeaturesHolder ImageContextHolder::get_features_holder() {
    return this->holder;
}

void ImageContextHolder::set_feature_holder(FeaturesHolder holder) {
    this->holder = holder;
}

cv::Mat ImageContextHolder::get_image() {
    return this->image_data;
}

std::string ImageContextHolder::get_image_id() {
    return this->image_id;
}

bool ImageContextHolder::is_image_in_memory() {
    return this->image_data.empty();
}


void ImageContextHolder::clear_image_data() {
    this->image_data.release();
}

FeaturesHolder FeatureExtractor::run_internal(ImageContextHolder& image_ctx) {
    INFOLOG("Running feature extract and compute engines");
    FeaturesHolder feature_holder;
    this->fextract->detect(
        image_ctx.get_image(), 
        feature_holder.kps
    );
    //cv::KeyPointsFilter::retainBest(feature_holder.kps, config.get_num_features_retained());
    this->fcompute->compute(
        image_ctx.get_image(), 
        feature_holder.kps,
        feature_holder.descriptors
    );
    image_ctx.set_feature_holder(feature_holder);
    return feature_holder;
}


FeaturesHolder FeatureExtractor::run(ImageContextHolder& image_ctx) {
    if(config.get_feature_extraction_algo() == FExtractionAlgorithm::ADAPTIVE_FAST) {
        return run_adaptive(image_ctx, 20);
    }
    return run_internal(image_ctx);
}

/*
Runs custom Adaptive FAST detector implementation, divides the image into W x H cells.
If max Nmax and min Nmin features is to be detected across the image, 


Nc_max = [Nmax / (W + H)], Nc_min = [Nmin / (W + H)] features must be extracted from each cell ideally. Each cell starts off with the same FAST threshold value.

Each iter for each cell,
ftc = fast threshold for that cell,
DETECT:
    feats = detect(ftc); 
    if Nd (Num features detected) < Nc_min, 
        threshold for that cell can be stepped down.
        goto DETECT
    else if Nd > Nc_max
        threshold stepped up
        goto DETECT
*/
FeaturesHolder FeatureExtractor::run_adaptive(ImageContextHolder& image_ctx, int r_depth) {
    FeaturesHolder holder = this->adaptive_extractor->extract(image_ctx);
    this->fcompute->compute(
        image_ctx.get_image(), 
        holder.kps,
        holder.descriptors
    );
    image_ctx.set_feature_holder(holder);
    return holder;
}

void FeatureExtractor::reset() {
    switch (config.get_descriptor_compute_algo()) {
        case vstk::DComputeAlgorithm::ORB :
            this->fcompute = cv::ORB::create();
            break;
        case vstk::DComputeAlgorithm::SIFT :
            this->fcompute = cv::SIFT::create();
            break;
        default:
            this->fcompute = cv::ORB::create();
            break;
    }

    switch (config.get_feature_extraction_algo()) {
        case vstk::FExtractionAlgorithm::FAST :
            this->fextract = cv::FastFeatureDetector::create(fast_threshold);
            break;
        case vstk::FExtractionAlgorithm::ORB :
            this->fextract = cv::ORB::create();
            break;
        case vstk::FExtractionAlgorithm::SIFT :
            this->fextract = cv::SIFT::create(500);
            break;
        case vstk::FExtractionAlgorithm::ADAPTIVE_FAST :
            this->adaptive_extractor = new AdaptiveFastExtractor(200, 2000, 4, 4);
            break;
        default:
            this->fextract = cv::FastFeatureDetector::create();
            break;
    }
}




FeatureExtractor::FeatureExtractor(vstk::VstkConfig config) : config(config) {
    this->reset();
}


void FeatureExtractor::display_features(ImageContextHolder image_ctx) {
    cv::Mat kp_im;
    cv::drawKeypoints(
        image_ctx.get_image(), 
        image_ctx.get_features_holder().kps, 
        kp_im, 
        cv::Scalar(0, 255, 0), 
        cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS
    );
    cv::imshow("keypoints", kp_im);
    cv::waitKey();
}

AdaptiveFastExtractor::AdaptiveFastExtractor(int n_min, int n_max, int cells_x, int cells_y) :
    n_min(n_min), 
    n_max(n_max), 
    cell_size(std::make_pair(cells_x, cells_y)) {
    this->fth_max = 100;
    this->fth_min = 10;
    this->th_step = 10;
    this->thresholds = std::vector<int>(16, 20);
    this->detectors = std::vector<cv::Ptr<cv::Feature2D>>(
        16,
        cv::FastFeatureDetector::create(20) 
    );
}

void AdaptiveFastExtractor::extract_internal(cv::Mat image, std::vector<cv::KeyPoint> &kps, int cell_idx, std::pair<int, int> origin) {
    this->detectors[cell_idx]->detect(
        image, 
        kps
    );
    for(cv::KeyPoint &kp : kps) {
        kp.pt.x += origin.first;
        kp.pt.y += origin.second;
    }
}

void AdaptiveFastExtractor::down_step_threshold(int cell_idx) {
    thresholds[cell_idx]-=th_step;
    if(thresholds[cell_idx] < fth_min) {
        thresholds[cell_idx] = fth_min;
    }
    detectors[cell_idx] = cv::FastFeatureDetector::create(thresholds[cell_idx]);
}

void AdaptiveFastExtractor::up_step_threshold(int cell_idx) {
    thresholds[cell_idx]+=th_step;
    if(thresholds[cell_idx] > fth_max) {
        thresholds[cell_idx] = fth_max;
    }
    detectors[cell_idx] = cv::FastFeatureDetector::create(thresholds[cell_idx]);
}

FeaturesHolder AdaptiveFastExtractor::extract(ImageContextHolder &im_ctx) {
    FeaturesHolder holder;
    cv::Mat im = im_ctx.get_image();
    std::vector<std::pair<int, int>> cell_origins;
    std::vector<cv::Mat> cells = vstk::split_image(im, 4, 4, cell_origins);
    int adj_iters = 0;
    int nc_max = n_max / cells.size();
    int nc_min = n_min / cells.size();

    if(nc_max < 1) nc_max = 1;
    if(nc_min < 1) nc_min = 1;
    DBGLOG("Adaptive detector : cells : %ld,  min_feature per cell : %d, max_feature per cell : %d", cells.size(), nc_min, nc_max);
    for(int i=0; i<cells.size(); i++) {
        std::vector<cv::KeyPoint> cell_kps;
        extract_internal(cells[i], cell_kps, i, cell_origins[i]);
        size_t n = cell_kps.size();
        DBGLOG("Detected %ld features in cell %d with threshold %d", n, i, thresholds[i]);
        while(n < nc_min && thresholds[i] > fth_min) {
            WARNLOG("Too few features in cell %d with threshold %d, decrementing cell threshold", i, thresholds[i]);
            down_step_threshold(i);
            WARNLOG("New threshold : %d", thresholds[i]);
            cell_kps.clear();
            extract_internal(cells[i], cell_kps, i, cell_origins[i]);
            n = cell_kps.size();
            adj_iters++;
        }

        while(n > nc_max && thresholds[i] < fth_max) {
            WARNLOG("Too many features in cell %d with threshold %d, incrementing cell threshold", i, thresholds[i]);
            up_step_threshold(i);
            cell_kps.clear();
            extract_internal(cells[i], cell_kps, i, cell_origins[i]);
            n = cell_kps.size();
            adj_iters++;
        }

        holder.kps.insert(std::end(holder.kps), std::begin(cell_kps), std::end(cell_kps));
    }
    INFOLOG("Adaptive FAST extractor summary : Total points : %ld, Threshold Adjustments : %d for 16 (4x4) cells", holder.kps.size(), adj_iters);
    return holder;
}
