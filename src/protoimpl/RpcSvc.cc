#ifdef VSTK_IO_GRPC

#include "protoimpl/RpcSvc.hpp"
#include "utils/AsyncUtils.hpp"
#include "core/Runtime.hpp"

using namespace vstk;

#include <iomanip>

enum class RT_SIGNAL {
    STOP,
    PAUSE,
    RESUME,
    INVALID
};

static RuntimeManager rt_man;

RT_SIGNAL get_rt_signal_enum(int rt_signal) {
    if(rt_signal == 0) {
        return RT_SIGNAL::STOP;
    }
    else if(rt_signal == 2) {
        return RT_SIGNAL::PAUSE;
    }
    else if(rt_signal == 3) {
        return RT_SIGNAL::RESUME;
    }
    else {
        return RT_SIGNAL::INVALID;
    }
}

void prettyPrintVector(const std::vector<double>& vec) {
    std::cout << "[";
    for (size_t i = 0; i < vec.size(); ++i) {
        std::cout << std::fixed << std::setprecision(2) << vec[i];
        if (i != vec.size() - 1) {
            std::cout << ", ";
        }
    }
    std::cout << "]" << std::endl;
}

bool is_dc_valid(std::shared_ptr<cv::Mat> _distCoeffs) {
    return (
        (_distCoeffs->rows == 1 || _distCoeffs->cols == 1) && (_distCoeffs->rows*_distCoeffs->cols == 4 || _distCoeffs->rows*_distCoeffs->cols == 5 || _distCoeffs->rows*_distCoeffs->cols == 8 || _distCoeffs->rows*_distCoeffs->cols == 12 || _distCoeffs->rows*_distCoeffs->cols == 14)
    );
}

cv::Mat vector_to_mat(const std::vector<double>& vec, int rows, int cols) {
    if (vec.size() != rows * cols) {
        INFOLOG("Size of the vector does not match the specified dimensions %ld != %ld", vec.size(), rows * cols);
        throw std::invalid_argument("Size of the vector does not match the specified dimensions");
    }
    cv::Mat mat(rows, cols, CV_64F);

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            mat.at<double>(i, j) = vec[i * cols + j];
        }
    }

    return mat;
}

int build_conf_from_grpc_req(const CreateStereoRTRequest *req, VstkConfig &conf) {
    std::string base_err = "Failed to initialize runtime config";
    if(req->data_src_left_stereo().empty() || req->data_src_right_stereo().empty()) {
        ERRORLOG("%s, one or more stereo source undefined", base_err);
        return EXIT_FAILURE;
    }

    if(req->camera_params().left_cam().k_matrix_size() == 0 || req->camera_params().right_cam().k_matrix_size() == 0) {
        ERRORLOG("%s, one or more intrinsic camera matrices undefined", base_err);
        return EXIT_FAILURE;
    }
    if(req->camera_params().r_matrix_size() == 0 || req->camera_params().t_vector_size() == 0) {
        ERRORLOG("%s, one or more extrinsic camera parameters undefined", base_err);
        return EXIT_FAILURE;
    }
    conf.set_stereo_src_1(req->data_src_left_stereo());
    conf.set_stereo_src_2(req->data_src_right_stereo());

    conf.set_feature_extraction_algo(
        vstk::extractor_algo_str_to_enum(
            req->feature_detector_config().detector_algorithm()
        )
    );

    conf.set_descriptor_compute_algo(
        vstk::desc_compute_algo_str_to_enum(
            req->feature_detector_config().compute_algorithm()
        )
    );

    conf.set_match_algo(
        vstk::matcher_algo_str_to_enum(
            req->feature_detector_config().matcher_algorithm()
        )
    );

    std::vector<double> kl_data;
    std::vector<double> kr_data; 
    std::vector<double> dl_data;
    std::vector<double> dr_data;
    std::vector<double> r_mat_data;
    std::vector<double> t_vec_data;
    for(int i=0; i < req->camera_params().left_cam().k_matrix_size(); i++) {
        kl_data.push_back(
            req->camera_params().left_cam().k_matrix(i)
        );
    }

    for(int i=0; i < req->camera_params().right_cam().k_matrix_size(); i++) {
        kr_data.push_back(
            req->camera_params().right_cam().k_matrix(i)
        );
    }

    for(int i=0; i < req->camera_params().left_cam().d_vector_size(); i++) {
        dl_data.push_back(
            req->camera_params().left_cam().d_vector(i)
        );
    }

    for(int i=0; i < req->camera_params().right_cam().d_vector_size(); i++) {
        dr_data.push_back(
            req->camera_params().right_cam().d_vector(i)
        );
    }

    for(int i=0; i < req->camera_params().r_matrix_size(); i++) {
        r_mat_data.push_back(
            req->camera_params().r_matrix(i)
        );
    }

    for(int i=0; i < req->camera_params().t_vector_size(); i++) {
        t_vec_data.push_back(
            req->camera_params().t_vector(i)
        );
    }

    cv::Mat kl_mat = vector_to_mat(kl_data, 3, 3);
    cv::Mat kr_mat = vector_to_mat(kr_data, 3, 3); 
    cv::Mat dl_mat = vector_to_mat(dl_data, 1, 5); 
    cv::Mat dr_mat = vector_to_mat(dr_data, 1, 5);; 
    cv::Mat r_mat = vector_to_mat(r_mat_data, 3, 3); 
    cv::Mat t_mat = vector_to_mat(t_vec_data, 1, 3);

    if(!is_dc_valid(std::make_shared<cv::Mat>(dl_mat)) || !is_dc_valid(std::make_shared<cv::Mat>(dl_mat))) {
        ERRORLOG("Invalid distortion coeffecient vector read");
    }

    MonoCamParams l_params, r_params;
    StereoCamParams stereo_params;

    l_params.K = kl_mat;
    l_params.dist_coeff = dl_mat;
    r_params.K = kr_mat;
    r_params.dist_coeff = dr_mat;

    stereo_params.cam1_params = l_params;
    stereo_params.cam2_params = r_params;
    stereo_params.Rs = r_mat;
    stereo_params.ts = t_mat;

    conf.set_stereo_cam_params(std::move(std::make_shared<vstk::StereoCamParams>(stereo_params)));
    conf.set_slam_type(vstk::SLAMType::STEREO);

    vstk::describe_config(conf);

    PersistenceConfig pconf;
    pconf.mode = PERSISTENCE_MODES::IN_PROCESS_MEMORY;
    conf.set_persistence_config(std::make_shared<PersistenceConfig>(pconf));
    return EXIT_SUCCESS;
}


grpc::Status VstkRPCService::CreateStereoRuntime(grpc::ServerContext* context, const CreateStereoRTRequest* request, CommonRTResponse* response) {
    INFOLOG("Received create runtime request, creating runtime...");
    vstk::set_headless(false);
    VstkConfig conf;
    if(build_conf_from_grpc_req(request, conf) != EXIT_SUCCESS) {
        ERRORLOG("Failed to build configuration");
        response->set_rt_state(-1);
        response->set_rt_id("<<INVALID>>");
        response->set_error_id("INITIALIZATION_ERR");
        response->set_error_message("Failed to initialize VSTK configuration for a runtime");
        return grpc::Status::OK;
    }
    std::string rt_id = rt_man.start_runtime(conf);
    INFOLOG("Successfully created Runtime with ID %s", rt_id);
    response->set_rt_state(0);
    response->set_rt_id(rt_id);
    return grpc::Status::OK;
}

grpc::Status VstkRPCService::ManageRuntime(grpc::ServerContext* context, const ManageRTRequest* request, CommonRTResponse* response) {
    INFOLOG("Received manage runtime request, managing runtime...");
    RT_SIGNAL rt_signal = get_rt_signal_enum(request->rt_signal());

    switch (rt_signal) {
        case RT_SIGNAL::STOP:
            rt_man.stop_runtime(request->rt_id());        
            break;
        default:
            break;
    }
    return grpc::Status::OK;
}

grpc::Status VstkRPCService::GetAllRTs(grpc::ServerContext* context, const GetAllRTSMessage* request, AllRTSResponse* response) {
    INFOLOG("Received [GetAllRTs] request ...");
    std::vector<std::string> all_rts = rt_man.get_all_running_rts();
    for(std::string rt_id : all_rts) {
        response->add_rt_id(rt_id);
    }
    INFOLOG("Successfully serviced [GetAllRTs]");
    return grpc::Status::OK;
}

grpc::Status VstkRPCService::GetRTStatus(grpc::ServerContext* context, const GetRTStatusMessage* request, RTStatusResponse* response) {
    INFOLOG("Received [GetRTStatus] request ...");

    INFOLOG("Successfully serviced [GetRTStatus]");
    return grpc::Status::OK;
}

#endif