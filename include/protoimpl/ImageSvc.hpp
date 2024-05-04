#include <stdio.h>

#ifdef VSTK_TRANSPORT_PROTO_GRPC
#include "utils/Logger.hpp"
#include "image_svc.grpc.pb.h"
#include "image_svc.pb.h"

#ifndef __IMG_SVC_PROTO_IMPL_H
#define __IMG_SVC_PROTO_IMPL_H


namespace vstk {

    class ImageSvc final : public X3DSService::Service {
        ::grpc::Status UploadImage(::grpc::ServerContext* context, ::grpc::ServerReader< ::UploadImageRequest>* reader, ::UploadImageResponse* response);
        ::grpc::Status ManageExchange(::grpc::ServerContext* context, const ::ManageExchangeRequest* request, ::ManageExchangeResponse* response);
    };

}
#endif

#endif