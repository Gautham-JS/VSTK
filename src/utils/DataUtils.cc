#include "utils/DataUtils.hpp"

using namespace vstk;

FPSDataPt_t vstk::create_fps_data_pt(std::string frame_id, double fps, bool is_insertion) {
    FPSDataPt_t data_pt;
    static int idx;
    
    data_pt.fps =           fps;
    data_pt.frame_id =      frame_id;
    data_pt.idx =           idx++;
    data_pt.is_insertion =  is_insertion;

    return data_pt;
}

void vstk::dump_data_pts_to_file(std::string filename, std::vector<vstk::FPSDataPt_t> data_pts) {
    INFOLOG("Dumping FPS data points to file %s", filename);
    cv::FileStorage fs = cv::FileStorage(filename, cv::FileStorage::WRITE);
    fs << "framerates" << "[";
    double avg_fps = 0;
    double sigma_fps = 0;
    for(FPSDataPt_t data : data_pts) {
        fs << "{";
        fs << "idx" << data.idx;
        fs << "frame_id" << data.frame_id;
        fs << "fps" << data.fps;
        fs << "is_insertion" << data.is_insertion; 
        fs << "}";
        sigma_fps += data.fps;
    }
    fs << "]";
    avg_fps = (sigma_fps / data_pts.size());
    fs << "average_fps" << avg_fps;
    fs.release();
    DBGLOG("File %s written successfully, data dump complete", filename);
}