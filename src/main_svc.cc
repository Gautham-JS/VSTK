#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>


#ifdef VSTK_IO_GRPC
    #include "grpcpp/grpcpp.h"
    #include "protoimpl/RpcSvc.hpp"
    #include "utils/Logger.hpp"

    using namespace vstk;

    void bindToGRPC(const std::string &addr) {
        INFOLOG("Attempting to bind using grpc to address %s", addr.c_str());
        
        vstk::Logger::get().set_level(VSTK_INFO);
        
        VstkRPCService image_service;
        grpc::ServerBuilder server_builder;

        INFOLOG("Server binding");
        server_builder.AddListeningPort(addr, grpc::InsecureServerCredentials());
        INFOLOG("Registering service");
        server_builder.RegisterService(&image_service);
        INFOLOG("Calling build and start");
        auto server = server_builder.BuildAndStart();
        INFOLOG("Server listening...");

        server->Wait();

        INFOLOG("Shut down server");
    }

#endif

int main(int argc, char** argv) {
    #ifdef VSTK_IO_GRPC
        bindToGRPC("0.0.0.0:11711");
    #else
        return EXIT_FAILURE;
    #endif
    return 0;
}