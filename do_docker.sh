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

IMAGE=$(sudo docker images | grep "rosndk " |  awk '{print $3}')
if [[ -z $IMAGE ]]; then
    echo -e '\e[34mCreting docker image.\e[39m'
    sudo docker build -t rosndk .
    # TODO: Verify successful docker image build
fi

if [ "x${1}" == "x--delete-image" ]
then
  echo
  echo -e '\e[34mDeleting docker image.\e[39m'
  echo
  sudo docker rmi -f rosndk

elif [ "x${1}" == "x--delete-tmp" ]
then
  echo
  echo -e '\e[34mDeleting docker temporary directory.\e[39m'
  echo
  sudo rm -rf /var/lib/docker/tmp

else
  echo
  echo -e '\e[34mRunning megabuild.\e[39m'
  echo
  echo -e '\e[34mSetting output_path to: '$output_path'.\e[39m'
  echo

  sudo docker run -t -v $my_loc:/opt/roscpp_android -v $output_path:/opt/roscpp_output -i rosndk /opt/roscpp_android/do_everything.sh /opt/roscpp_output

fi
