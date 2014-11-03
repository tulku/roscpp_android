#!/bin/bash

my_loc="$(cd "$(dirname $0)" && pwd)"
source $my_loc/config.sh
source $my_loc/utils.sh

if [ $# != 1 ] || [ $1 == '-h' ] || [ $1 == '--help' ]; then
    echo "Usage: $0 tinyxml_source_dir"
    echo "  example: $0 /home/user/my_workspace/lz4"
    exit 1
fi

cmake_build $1

cp $my_loc/../roscpp_output/libs/lz4-r123/lz4.h $my_loc/../roscpp_output/target/include/
cp $my_loc/../roscpp_output/libs/lz4-r123/lz4hc.h $my_loc/../roscpp_output/target/include/
