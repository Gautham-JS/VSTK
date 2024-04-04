cmake_minimum_required(VERSION 3.10)

set(CMAKE_C_COMPILER "gcc")
set(CMAKE_CXX_COMPILER "g++")

project( vstk-core )

set(ABSL_ENABLE_INSTALL ON)
set(gRPC_RE2_PROVIDER "package")

set (WITH_OPENGL ON)
set (WITH_OPENCL ON)
set (WITH_PROTOBUF OFF)
set (OPENCV_GENERATE_PKGCONFIG ON)
set (OPENCV_EXTRA_MODULES_PATH ${CMAKE_SOURCE_DIR}/thirdparty/opencv/opencv_contrib/modules)
set (BUILD_SHARED_LIBS ON)


add_subdirectory(${CMAKE_SOURCE_DIR}/thirdparty/grpc/grpc/ ${CMAKE_BINARY_DIR}/thirdparty/grpc/)


list(APPEND CMAKE_PREFIX_PATH "${CMAKE_SOURCE_DIR}/thirdparty/opencv/install/")
list(APPEND CMAKE_LIBRARY_PATH "${CMAKE_SOURCE_DIR}/thirdparty/opencv/install/lib/")

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)


set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(PROTO_SOURCES ${CMAKE_SOURCE_DIR}/protobuf/image_svc.proto)

set(OpenCV_DIR "${CMAKE_SOURCE_DIR}/thirdparty/opencv/build/")


find_package( OpenCV REQUIRED )
find_package( Boost REQUIRED COMPONENTS log)

# Proto file
get_filename_component(svc_proto "${CMAKE_SOURCE_DIR}/protobuf/image_svc.proto" ABSOLUTE)
get_filename_component(svc_proto_path "${svc_proto}" PATH)

# Generated sources
set(svc_proto_srcs "${CMAKE_SOURCE_DIR}/protos/image_svc.pb.cc")
set(svc_proto_hdrs "${CMAKE_SOURCE_DIR}/protos/image_svc.pb.h")
set(svc_grpc_srcs "${CMAKE_SOURCE_DIR}/protos/image_svc.grpc.pb.cc")
set(svc_grpc_hdrs "${CMAKE_SOURCE_DIR}/protos/image_svc.grpc.pb.h")
set(_PROTOBUF_LIBPROTOBUF protobuf::libprotobuf)
set(_GRPC_GRPCPP_UNSECURE grpc++_unsecure)

if(CMAKE_CROSSCOMPILING)
  find_program(_PROTOBUF_PROTOC protoc)
else()
  set(_PROTOBUF_PROTOC $<TARGET_FILE:protobuf::protoc>)
endif()
set(_GRPC_GRPCPP grpc++)
if(CMAKE_CROSSCOMPILING)
  find_program(_GRPC_CPP_PLUGIN_EXECUTABLE grpc_cpp_plugin)
else()
  set(_GRPC_CPP_PLUGIN_EXECUTABLE $<TARGET_FILE:grpc_cpp_plugin>)
endif()

add_custom_command(
  OUTPUT "${svc_proto_srcs}" "${svc_proto_hdrs}" "${svc_grpc_srcs}" "${svc_grpc_hdrs}"
  COMMAND ${_PROTOBUF_PROTOC}
  ARGS --grpc_out "${CMAKE_SOURCE_DIR}/protos"
    --cpp_out "${CMAKE_SOURCE_DIR}/protos"
    -I "${svc_proto_path}"
    --plugin=protoc-gen-grpc=${_GRPC_CPP_PLUGIN_EXECUTABLE}
    "${svc_proto}"
  DEPENDS "${svc_proto}"
)

include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories( ${CMAKE_SOURCE_DIR}/protos/ )

include_directories(
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
  "${CMAKE_SOURCE_DIR}/thirdparty/opencv/install/include/opencv4/"
  "${CMAKE_SOURCE_DIR}/thirdparty/grpc/install/include/"
) 


add_library(svc_grpc_proto SHARED
  ${svc_grpc_srcs}
  ${svc_grpc_hdrs}
  ${svc_proto_srcs}
  ${svc_proto_hdrs}
)
target_link_libraries( svc_grpc_proto grpc++ grpc++_unsecure libprotobuf)

add_library(
    features_vstk SHARED
    src/features/FeatureExtractor.cpp
    src/features/FeatureMatcher.cpp
)
target_link_libraries( features_vstk ${Boost_LIBRARIES})

add_library(  
    image_svc_vstk SHARED
    src/protoimpl/ImageSvc.cpp
)
target_link_libraries( image_svc_vstk grpc++ grpc++_unsecure libprotobuf ${Boost_LIBRARIES})

add_library(
    utils_vstk SHARED
    src/utils/ObjectLifecycle.cpp
)

target_link_libraries(utils_vstk  ${Boost_LIBRARIES})



add_executable( 3ds src/main.cpp )



message( "OPENCV LIBS :: ${Boost_LIBRARIES}" )
message( "BOOST INCL :: ${Boost_LOG_LIBRARY}" )



target_link_libraries( 3ds ${Boost_LIBRARIES})

target_link_libraries( 3ds features_vstk utils_vstk )
target_link_libraries( 3ds svc_grpc_proto image_svc_vstk )
target_link_libraries( 3ds ${OpenCV_LIBS})
target_link_libraries( 3ds grpc++ grpc++_unsecure )

