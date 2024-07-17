#include "protoimpl/RpcSvc.hpp"


using namespace vstk;


grpc::Status CreateStereoRuntime(grpc::ServerContext* context, const CreateStereoRTRequest* request, CommonRTResponse* response) {
    INFOLOG("Received create runtime request, creating runtime...");
    

    INFOLOG("Successfully created Runtime");
    return grpc::Status::OK;
}

// void log_upload_image_request(UploadImageRequest req) {
//     INFOLOG("Reading data...");
//     INFOLOG("Image data : %s", std::string(req.chunk_data()).c_str());
//     INFOLOG("Image ID : %s", req.image_id().c_str());
//     INFOLOG("Image Type : %s", req.image_type().c_str());
//     INFOLOG("Exchange ID : %s", req.exchange_id().c_str());
//     INFOLOG("Client ID : %s", req.client_id().c_str());
//     INFOLOG("Image Number : %s", req.image_number());
//     INFOLOG("Image Color Space : %s", req.image_color_space().c_str());

//     if(req.has_chunk_number()) INFOLOG("Chunk Number : %s", req.chunk_number());
//     if(req.has_chunk_id()) INFOLOG("Chunk ID : %s", req.chunk_id().c_str());
//     if(req.has_start_chunk_number()) INFOLOG("Chunk Start Number : %d", req.start_chunk_number());
//     if(req.has_end_chunk_number()) INFOLOG("Chunk End Number : %d", req.end_chunk_number());
// }

// void log_manage_exchange_req(ManageExchangeRequest req) {
//     INFOLOG("State : %s", req.state().c_str());
    
//     if(req.has_io_scheme()) INFOLOG("IO Scheme : %s", req.io_scheme().c_str());
//     if(req.has_processing_scheme()) INFOLOG("IO Scheme : %s", req.processing_scheme().c_str());
//     if(req.has_exchange_id()) INFOLOG("Exchange ID : %s", req.exchange_id().c_str());
//     if(req.has_start_image_number()) INFOLOG("Start Image Number : %d", req.start_image_number());
//     if(req.has_end_image_number()) INFOLOG("End Image Number : %d", req.end_image_number());
// }

// grpc::Status ImageSvc::UploadImage(grpc::ServerContext* context, grpc::ServerReader<UploadImageRequest>* reader, UploadImageResponse* response) {
//     INFOLOG("Received upload image request");
//     UploadImageRequest req;
//     INFOLOG("Stream loop : start");
//     while(reader->Read(&req)) {
//         log_upload_image_request(req);
//     }
//     INFOLOG("Stream loop : end");
//     response->set_client_id("1234");
//     return grpc::Status::OK;
// }


// grpc::Status ImageSvc::ManageExchange(grpc::ServerContext* context, const ManageExchangeRequest* request, ManageExchangeResponse* response) {
//     INFOLOG("Received manage exchange request");
//     log_manage_exchange_req(*request);
//     return grpc::Status::OK;
// }
