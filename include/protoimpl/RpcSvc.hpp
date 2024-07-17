#include <stdio.h>

#include "utils/Logger.hpp"

#include "image_svc.grpc.pb.h"
#include "image_svc.pb.h"

#ifndef __IMG_SVC_PROTO_IMPL_H
#define __IMG_SVC_PROTO_IMPL_H


namespace vstk {

    // class ImageSvc final : public X3DSService::Service {
    //     ::grpc::Status UploadImage(::grpc::ServerContext* context, ::grpc::ServerReader< ::UploadImageRequest>* reader, ::UploadImageResponse* response);
    //     ::grpc::Status ManageExchange(::grpc::ServerContext* context, const ::ManageExchangeRequest* request, ::ManageExchangeResponse* response);
    // };

    class VstkRPCService final : public VstkService::Service {
        public:
            grpc::Status CreateStereoRuntime(grpc::ServerContext* context, const CreateStereoRTRequest* request, CommonRTResponse* response);
            grpc::Status ManageRuntime(grpc::ServerContext* context, const ManageRTRequest* request, CommonRTResponse* response);
            grpc::Status GetAllRTs(grpc::ServerContext* context, const GetAllRTSMessage* request, AllRTSResponse* response);
            grpc::Status GetRTStatus(grpc::ServerContext* context, const GetRTStatusMessage* request, RTStatusResponse* response);
    };
}

#endif