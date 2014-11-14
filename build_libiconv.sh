#!/bin/bash

my_loc="$(cd "$(dirname $0)" && pwd)"
source $my_loc/config.sh
source $my_loc/utils.sh

if [ $# != 1 ] || [ $1 == '-h' ] || [ $1 == '--help' ]; then
    echo "Usage: $0 boost_source_dir"
    echo "  example: $0 /home/user/my_workspace/libiconv-1.14"
    exit 1
fi

prefix=$(cd $1 && pwd)

cd $1

# Create a stand alone version of the android toolchain
echo
echo -e '\e[34mBuilding libiconv.\e[39m'
echo

[ "$CMAKE_PREFIX_PATH" = "" ] && die 'could not find target basedir. Have you run build_catkin.sh and sourced setup.bash?'

if [ ! -d toolchain/ ]; then
  mkdir toolchain/
  $ANDROID_NDK/build/tools/make-standalone-toolchain.sh --platform=android-8 --install-dir=./toolchain --ndk-dir=$ANDROID_NDK --system=linux-x86_64
fi
./configure --prefix=$CMAKE_PREFIX_PATH --enable-shared=no --enable-static
export PATH=$PATH:$1/toolchain/bin
make -s -j8
make install