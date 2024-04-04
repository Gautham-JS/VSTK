#include <stdio.h>

#include "utils/Logger.hpp"
#include "image_svc.grpc.pb.h"
#include "image_svc.pb.h"

#ifndef __IMG_SVC_PROTO_IMPL_H
#define __IMG_SVC_PROTO_IMPL_H


namespace x3ds {

    class ImageSvc final : public X3DSService::Service {
        ::grpc::Status UploadImage(::grpc::ServerContext* context, ::grpc::ServerReader< ::UploadImageRequest>* reader, ::UploadImageResponse* response);
        ::grpc::Status ManageExchange(::grpc::ServerContext* context, const ::ManageExchangeRequest* request, ::ManageExchangeResponse* response);
    };

}


#endif