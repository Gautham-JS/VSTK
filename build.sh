

function fatal {
    printf "\n\n\t[X] ERROR : $1\n";
    exit 1;
}

echo "Building VSTK"
./thirdparty/install_prerequisites.sh || fatal "Failed to install VSTK prerequisites"
./thirdparty/build_thirdparty.sh install || fatal "Failed to build VSTK prerequisites"
rm -rf ./build; 
mkdir build && cd build
cmake .. || fatal "Failed to configure project VSTK"
make -j15 || fatal "Failed to build project"
