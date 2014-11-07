#!/bin/bash

my_loc="$(cd "$(dirname $0)" && pwd)"
source $my_loc/config.sh
source $my_loc/utils.sh

if [ $# != 1 ] || [ $1 == '-h' ] || [ $1 == '--help' ]; then
    echo "Usage: $0 prefix_path"
    echo "  example: $0 /home/user/my_workspace"
    exit 1
fi

prefix=$(cd $1 && pwd)
URL=https://github.com/Cyan4973/lz4/archive/r123.tar.gz

echo
echo -e '\e[34mGetting lz4.\e[39m'
echo

download_gz $URL $prefix
