#!/bin/bash

my_loc="$(cd "$(dirname $0)" && pwd)"
source $my_loc/utils.sh

if [[ $# -lt 1 ]] ; then
    output_path=$my_loc"/output"
else
    output_path=$1
fi

# Requires docker 1.3+
cmd_exists docker || die 'docker was not found'

echo
echo -e '\e[34mBuilding docker image.\e[39m'
echo

# Build docker image
sudo docker build -t rosndk .
# TODO: Verify successful docker image build

echo
echo -e '\e[34mRunning megabuild.\e[39m'
echo
echo -e '\e[34mSetting output_path to: '$output_path'.\e[39m'
echo

sudo docker run -t -v $my_loc:/opt/roscpp_android -v $output_path:/opt/roscpp_output -i rosndk /opt/roscpp_android/do_everything.sh /opt/roscpp_output
