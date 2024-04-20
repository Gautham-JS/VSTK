
export TP_BUILD_ROOT="$(pwd)/thirdparty"


GRPC_VER="1.62.1"
OPENCV_VER="4.9.0"
BOOST_VER="1.83.0"

MODULES="*"
IS_OPENCV_SELECTED=1
IS_GRPC_SELECTED=1





function check_dir {
    if test -d $1; then
        return 0
    fi
    return 1
}

function clean {
  if check_dir "$TP_BUILD_ROOT/opencv" && check_module "opencv"; then
    printf  "Removing OpenCV\n"
    eval "rm -rf $TP_BUILD_ROOT/opencv/build"
    eval "rm -rf $TP_BUILD_ROOT/opencv/install"
  fi
  if check_dir "$TP_BUILD_ROOT/grpc" && check_module "grpc"; then
    printf "Removing GRPC\n"
    eval "rm -rf $TP_BUILD_ROOT/grpc/build"
    eval "rm -rf $TP_BUILD_ROOT/grpc/install"

  fi
}

function assert_rc {
  if [ $1 -ne 0 ]; then
    echo "$2"
    exit -1;
  fi
}

function install_opencv {
  #eval "sudo apt install build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev python3-pip python3-numpy"
  #assert_rc $? "Failed to install opencv dependencies"
  if ! check_module "opencv" ; then
    echo "Opencv not selected for installation, skipping"
    return
  fi
  if check_dir "$TP_BUILD_ROOT/opencv"; then
    echo "OpenCV directory already exists"
    eval "pushd opencv"
  else
    echo "Creating directory for building OpenCV"
    eval "mkdir opencv && pushd opencv"
    eval "wget --no-check-certificate -O opencv.zip https://github.com/opencv/opencv/archive/$OPENCV_VER.zip && wget --no-check-certificate -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/$OPENCV_VER.zip"
    assert_rc $? "Failed to get opencv / opencv contrib modules"
    unzip opencv.zip && unzip opencv_contrib.zip
    assert_rc $? "Failed to unpack opencv / opencv contrib modules"
    mv ./opencv-$OPENCV_VER/ ./opencv/ && mv ./opencv_contrib-$OPENCV_VER/ ./opencv_contrib/
    assert_rc $? "Failed to rename opencv directories"
  fi
  
  if check_dir "$TP_BUILD_ROOT/opencv/build"; then
    cd "$TP_BUILD_ROOT/opencv/build"
  else
    mkdir -p "$TP_BUILD_ROOT/opencv/build"
    cd "$TP_BUILD_ROOT/opencv/build"
  fi
  pwd
  cmake -D WITH_CUDA=OFF \
    -D BUILD_TIFF=ON \
    -D BUILD_opencv_java=OFF \
    -D WITH_OPENGL=ON \
    -D WITH_OPENCL=ON \
    -D WITH_IPP=ON \
    -D WITH_TBB=ON \
    -D WITH_EIGEN=ON \
    -D WITH_V4L=ON \
    -D WITH_VTK=OFF \
    -D BUILD_TESTS=OFF \
    -D BUILD_PERF_TESTS=OFF \
    -D CMAKE_BUILD_TYPE=DEBUG \
    -D BUILD_opencv_python2=OFF \
    -D BUILD_opencv_python3=OFF \
    -D CMAKE_INSTALL_PREFIX=$TP_BUILD_ROOT/opencv/install \
    -D INSTALL_C_EXAMPLES=OFF \
    -D INSTALL_PYTHON_EXAMPLES=OFF \
    -D OPENCV_ENABLE_NONFREE=OFF \
    -D OPENCV_GENERATE_PKGCONFIG=ON \
    -D OPENCV_EXTRA_MODULES_PATH=$TP_BUILD_ROOT/opencv/opencv_contrib/modules \
    -D BUILD_EXAMPLES=ON \
    $TP_BUILD_ROOT/opencv/opencv/
  if [ $? -eq 0 ] 
  then 
    echo "Successfully run cmake on opencv" 
    #exit 0 
  else 
    echo "Could not cmake opencv project" >&2 
    exit 1 
  fi
  make -j$(( $(nproc) - 1))
  assert_rc $? "Failed to compile OpenCV source"
  make install 
  assert_rc $? "Failed to install OpenCV locally"
  popd
}


function install_grpc {
  # if ! check_module "grpc" ; then
  #   echo "gRPC not selected for installation, skipping"
  #   return
  # fi
  echo "Installing gRPC"
  export GRPC_INSTALL_DIR="$TP_BUILD_ROOT/grpc/install"
  if check_dir "$TP_BUILD_ROOT/grpc"; then
    echo "gRPC directory already exists"
    eval "pushd grpc"
  else
    echo "Creating directory for building gRPC"
    eval "mkdir grpc && pushd grpc"
    eval "git clone --recurse-submodules -b v$GRPC_VER --depth 1 --shallow-submodules https://github.com/grpc/grpc"
    assert_rc $? "Failed to clone gRPC"
  fi

  # if check_dir "$TP_BUILD_ROOT/grpc/build"; then
  #   cd "$TP_BUILD_ROOT/grpc/build"
  # else
  #   mkdir -p "$TP_BUILD_ROOT/grpc/build"
  #   cd "$TP_BUILD_ROOT/grpc/build"
  # fi

  #  echo "Building in $(pwd)"
  #  cmake -D gRPC_INSTALL=ON \
  #    -D gRPC_BUILD_TESTS=OFF \
  #    -D BUILD_SHARED_LIBS=ON \
  #    -D CMAKE_INSTALL_PREFIX=$GRPC_INSTALL_DIR \
  #    -D CMAKE_BUILD_TYPE=Release \
  #    $TP_BUILD_ROOT/grpc/grpc
  #  assert_rc $? "Failed to CMake gRPC"

  # make -j$(( $(nproc) - 1 ))
  # assert_rc $? "Failed to compile gRPC"

  # make install
  # assert_rc $? "Failed to install gRPC locally"

  popd
}

function install_boost {
  echo "Installing Boost"
  if check_dir "$TP_BUILD_ROOT/boost"; then
    echo "Boost directory already exists"
    eval "pushd boost"
  else
    echo "Creating directory for building Boost"
    eval "mkdir boost && pushd boost"
    eval "git clone --recursive https://github.com/boostorg/boost.git"
    assert_rc $? "Failed to get boost source"
  fi

  cd boost
  DST_DIR="$TP_BUILD_ROOT/boost/install"
  echo $DST_DIR
  ./bootstrap.sh --prefix=$DST_DIR --with-toolset=gcc --show-libraries --includedir=$DST_DIR/include --libdir=$DST_DIR/lib
  assert_rc $? "Failed to configure boost source"
  ./b2 install --prefix=$DST_DIR
  assert_rc $? "Failed to install boost"

  popd
}


function install {
  install_opencv
  install_grpc
}


function usage() {
  usage="build_thirdparty.sh <operation> <module (optional)>" 
  echo "$usage"
}


function check_module {
  if $MODULES eq "*"; then
    return 1
  fi
  if echo "$MODULES" | grep -q "\\<$1\\>"; then
    return 1;
  else
    return 0;
  fi
}






if [ -z "$1" ]; then
    echo "No argument supplied."
    usage
    exit -1
fi

if [ -z "$2" ]; then
    echo "Selected all modules."
else
    MODULES="'$*'"
    echo "Selected module(s) : $MODULES"
fi



pushd $TP_BUILD_ROOT
case $1 in
  install)
    echo "Installation operation"
    install
    ;;
  clean)
    echo "Clean operation"
    clean
    ;;
  *)
    usage
    exit -1;
    ;;
esac
popd



