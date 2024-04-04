
FAIL_MSG="Failed to install dependencies"
OK_MSG="Succesfully installed dependencies"

function run_cmd() {
	if ! eval $1; then
		echo $FAIL_MSG
		exit -1
	fi

}

function opencv_deps() {
	run_cmd "sudo add-apt-repository -y 'deb http://archive.ubuntu.com/ubuntu/ jammy-updates main restricted universe multiverse'"
	run_cmd "sudo add-apt-repository -y 'deb http://archive.ubuntu.com/ubuntu/ jammy-security main restricted universe multiverse'"
	run_cmd "sudo apt-get install -y build-essential"
	run_cmd "sudo apt-get install -y cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev"
	run_cmd "sudo apt-get install -y python3 libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev" 
}

function grpc_deps() {
	run_cmd "sudo apt-get install -y build-essential autoconf libtool pkg-config" 
	run_cmd "sudo apt-get install -y libgflags-dev libgtest-dev"
	run_cmd "sudo apt-get install -y clang libc++-dev"
	run_cmd "sudo apt-get install -y libboost-all-dev"
}


opencv_deps
grpc_deps
