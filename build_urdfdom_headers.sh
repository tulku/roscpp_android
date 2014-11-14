#!/bin/bash

my_loc="$(cd "$(dirname $0)" && pwd)"
source $my_loc/config.sh
source $my_loc/utils.sh

if [ $# != 1 ] || [ $1 == '-h' ] || [ $1 == '--help' ]; then
    echo "Usage: $0 tinyxml_source_dir"
    echo "  example: $0 /home/user/my_workspace/urdfdom_headers"
    exit 1
fi

echo
echo -e '\e[34mBuilding urdfdom_headers.\e[39m'
echo

cmake_build $1

