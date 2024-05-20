cmake_minimum_required(VERSION 3.12)

set(CMAKE_C_COMPILER "gcc")
set(CMAKE_CXX_COMPILER "g++")

project( vstk )

if(WALL)
  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
  endif()
endif()

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

set(ABSL_ENABLE_INSTALL ON)
set(gRPC_RE2_PROVIDER "module")

set (WITH_OPENGL ON)
set (WITH_OPENCL ON)
set (WITH_PROTOBUF OFF)
set (OPENCV_GENERATE_PKGCONFIG ON)
set (OPENCV_EXTRA_MODULES_PATH ${CMAKE_SOURCE_DIR}/thirdparty/opencv/opencv_contrib/modules)
set (BUILD_SHARED_LIBS ON)


message("${CMAKE_SOURCE_DIR}/thirdparty/grpc/grpc/")

# 

list(APPEND CMAKE_PREFIX_PATH "${CMAKE_SOURCE_DIR}/thirdparty/grpc/install/")
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_SOURCE_DIR}/thirdparty/opencv/install/")
list(APPEND CMAKE_LIBRARY_PATH "${CMAKE_SOURCE_DIR}/thirdparty/opencv/install/lib/")
list(APPEND CMAKE_LIBRARY_PATH "${CMAKE_SOURCE_DIR}/thirdparty/grpc/install/lib/")


set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
link_directories("${CMAKE_SOURCE_DIR}/thirdparty/grpc/install/lib/")

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(PROTO_SOURCES ${CMAKE_SOURCE_DIR}/protobuf/image_svc.proto)

set(OpenCV_DIR "${CMAKE_SOURCE_DIR}/thirdparty/opencv/build/")
set(gRPC_DIR "${CMAKE_SOURCE_DIR}/thirdparty/grpc/build/**")


find_package( OpenCV REQUIRED )
find_package( Boost REQUIRED COMPONENTS log filesystem)
if(TRANSPORT_GRPC)
  if(USE_SYSTEM_GRPC)
    find_package( Protobuf REQUIRED)
    find_package( gRPC REQUIRED )
    find_program(_PROTOBUF_PROTOC protoc)
    find_program(_GRPC_CPP_PLUGIN_EXECUTABLE grpc_cpp_plugin)
  else()
    add_subdirectory(${CMAKE_SOURCE_DIR}/thirdparty/grpc/grpc/)
    set(_PROTOBUF_PROTOC $<TARGET_FILE:protobuf::protoc>)
    set(_GRPC_CPP_PLUGIN_EXECUTABLE $<TARGET_FILE:grpc_cpp_plugin>)
  endif()
  set(svc_proto_srcs "${CMAKE_SOURCE_DIR}/protos/image_svc.pb.cc")
  set(svc_proto_hdrs "${CMAKE_SOURCE_DIR}/protos/image_svc.pb.h")
  set(svc_grpc_srcs "${CMAKE_SOURCE_DIR}/protos/image_svc.grpc.pb.cc")
  set(svc_grpc_hdrs "${CMAKE_SOURCE_DIR}/protos/image_svc.grpc.pb.h")
  set(_PROTOBUF_LIBPROTOBUF protobuf::libprotobuf)
  set(_GRPC_GRPCPP_UNSECURE grpc++_unsecure)
  set(_GRPC_GRPCPP grpc++)

  add_compile_definitions(VSTK_TRANSPORT_PROTO_GRPC)
endif()

# Proto file
get_filename_component(svc_proto "${CMAKE_SOURCE_DIR}/protobuf/image_svc.proto" ABSOLUTE)
get_filename_component(svc_proto_path "${svc_proto}" PATH)

# Generated sources


function(check_lib libname libsymbol)
  if(NOT DEFINED ${libsymbol}) 
	  message(FATAL_ERROR "${libname} Libraries : [UNDEFINED]")
  endif()
  message( "${libname} Libraries ------ [OK]")
endfunction()

function(dump_cmake_variables)
    get_cmake_property(_variableNames VARIABLES)
    list (SORT _variableNames)
    foreach (_variableName ${_variableNames})
        if ((NOT DEFINED ARGV0) OR _variableName MATCHES ${ARGV0})
            message(STATUS "${_variableName}=${${_variableName}}")
        endif()
    endforeach()
endfunction()


include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories( ${Protobuf_INCLUDE_DIRS} )
include_directories( ${CMAKE_SOURCE_DIR}/protos/ )

include_directories(
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
  "${CMAKE_SOURCE_DIR}/thirdparty/opencv/install/include/opencv4/"
  "${CMAKE_SOURCE_DIR}/thirdparty/grpc/install/include/"
) 




# if (NOT DEFINED OpenCV_LIBS)
# 	message(FATAL_ERROR "OpenCV Libraries : [UNDEFINED]")
# endif()
# message("OpenCV Libraries : [OK]")

# if(NOT DEFINED Boost_LIBRARIES) 
# 	message(FATAL_ERROR "Boost Libraries : [UNDEFINED]")
# endif()
# message( "Boost Libraries : [OK]")

# if(NOT DEFINED Protobuf_INCLUDE_DIRS)
#   message( "Protocol Buffer Libraries : [UNDEFINED]" )
# endif()
# message( "Protocol Buffer Libraries : [OK] :: ${Protobuf_INCLUDE_DIRS}" )

message("\n\n")
message("==> SYSTEM LIBRARY CHECKS\n")
check_lib("OpenCV" "OpenCV_LIBS")
check_lib("Boost" "Boost_LIBRARIES")
if(TRANSPORT_GRPC)
  if(USE_SYSTEM_GRPC)
    check_lib("Protocol Buffer" "_PROTOBUF_LIBPROTOBUF")
    check_lib("gRPC C++" _GRPC_GRPCPP)
  else()
    message("Protocol Buffer ------ [QUEUED FOR BUILD]")
    message("gRPC C++ ------ [QUEUED FOR BUILD]")
  endif()
endif()

message("\n== SYSTEM LIBRARY CHECK COMPLETE\n")

if(TRANSPORT_GRPC)
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
  add_library(svc_grpc_proto SHARED
    ${svc_grpc_srcs}
    ${svc_grpc_hdrs}
    ${svc_proto_srcs}
    ${svc_proto_hdrs}
  )
  target_link_libraries( svc_grpc_proto  grpc++ grpc++_unsecure libprotobuf)

  add_library(  
    image_svc_vstk SHARED
    src/protoimpl/ImageSvc.cc
  )
  target_link_libraries( image_svc_vstk  grpc++ grpc++_unsecure libprotobuf ${Boost_LIBRARIES})
endif()

add_library(
    features_vstk SHARED
    src/features/FeatureExtractor.cc
    src/features/FeatureMatcher.cc
)
target_link_libraries( features_vstk ${Boost_LIBRARIES})

add_library(
    config_vstk SHARED
    src/config/Config.cc
)
target_link_libraries(config_vstk  ${Boost_LIBRARIES})

add_library(
    io_vstk SHARED
    src/io/DiskIO.cc
    src/io/Serializer.cc
)

target_link_libraries(io_vstk  ${Boost_LIBRARIES})

add_library(
    utils_vstk SHARED
    src/utils/ObjectLifecycle.cc
    src/utils/TimerUtils.cc
    src/utils/CvUtils.cc
)
target_link_libraries(utils_vstk  ${Boost_LIBRARIES})

add_library(
    calib_kit_vstk SHARED
    src/calib/CameraCalibration.cc
    src/config/CalibConfig.cc
)
target_link_libraries(calib_kit_vstk  config_vstk ${Boost_LIBRARIES})



add_executable( vstk src/main.cc )

target_link_libraries( vstk ${Boost_LIBRARIES})

target_link_libraries( vstk features_vstk utils_vstk io_vstk config_vstk)
target_link_libraries( vstk ${OpenCV_LIBS})

add_executable( calib_kit src/calib.cc )
target_link_libraries( calib_kit ${Boost_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries( calib_kit calib_kit_vstk)

if(TRANSPORT_GRPC)
  target_link_libraries( vstk svc_grpc_proto image_svc_vstk )
  target_link_libraries( vstk grpc++ grpc++_unsecure libprotobuf )
endif()
