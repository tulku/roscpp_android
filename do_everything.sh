#!/bin/bash

# TODO: Check that environment actually exists
source /opt/ros/indigo/setup.bash

my_loc="$(cd "$(dirname $0)" && pwd)"
source $my_loc/config.sh
source $my_loc/utils.sh
debugging=0
skip=0
help=0

if [[ $# -lt 1 ]] ; then
    help=1
fi

for var in "$@"
do
    if [[ ${var} == "--help" ]] ||  [[ ${var} == "-h" ]] ; then
        help=1
    fi
    if [[ ${var} == "--skip" ]] ; then
        skip=1
    fi

    if [[ ${var} == "--debug-symbols" ]] ; then
        debugging=1
    fi
done

if [[ $help -eq 1 ]] ; then
    echo "Usage: $0 prefix_path [-h | --help] [--skip] [--debug-symbols]"
    echo "  example: $0 /home/user/my_workspace"
    exit 1
fi

if [[ $skip -eq 1 ]]; then
   echo "-- Skiping projects update"
else
   echo "-- Will update projects"
fi

if [[ $debugging -eq 1 ]]; then
   echo "-- Building workspace WITH debugging symbols"
else
   echo "-- Building workspace without debugging symbols"
fi

if [ ! -d $1 ]; then
    mkdir -p $1
fi

prefix=$(cd $1 && pwd)

run_cmd() {
    cmd=$1.sh
    shift
    $my_loc/$cmd $@ || die "$cmd $@ died with error code $?"
}

if [ -z $ANDROID_NDK ] ; then
    die "ANDROID_NDK ENVIRONMENT NOT FOUND!"
fi

if [ -z $ROS_DISTRO ] ; then
    die "HOST ROS ENVIRONMENT NOT FOUND! Did you source /opt/ros/indigo/setup.bash"
fi

[ -d $standalone_toolchain_path ] || run_cmd setup_standalone_toolchain

echo
echo -e '\e[34mGetting library dependencies.\e[39m'
echo

mkdir -p $prefix/libs

# Start with catkin since we use it to build almost everything else
[ -d $prefix/target ] || mkdir -p $prefix/target
export CMAKE_PREFIX_PATH=$prefix/target

[ -e $prefix/android.toolchain.cmake ] || ( cd $prefix && download 'https://raw.github.com/taka-no-me/android-cmake/master/android.toolchain.cmake' && cat $my_loc/files/android.toolchain.cmake.addendum >> $prefix/android.toolchain.cmake)
export RBA_TOOLCHAIN=$prefix/android.toolchain.cmake

# Now get boost with a specialized build
[ -d $prefix/libs/boost ] || run_cmd get_boost $prefix/libs
[ -d $prefix/libs/bzip2 ] || run_cmd get_bzip2 $prefix/libs
[ -d $prefix/libs/uuid ] || run_cmd get_uuid $prefix/libs
[ -d $prefix/libs/poco-1.4.6p2 ] || run_cmd get_poco $prefix/libs
[ -d $prefix/libs/tinyxml ] || run_cmd get_tinyxml $prefix/libs
[ -d $prefix/libs/catkin ] || run_cmd get_catkin $prefix/libs
[ -d $prefix/libs/console_bridge ] || run_cmd get_console_bridge $prefix/libs
[ -d $prefix/libs/lz4-r124 ] || run_cmd get_lz4 $prefix/libs
[ -d $prefix/libs/curl-7.39.0 ] || run_cmd get_curl $prefix/libs
[ -d $prefix/libs/urdfdom/ ] || run_cmd get_urdfdom $prefix/libs
[ -d $prefix/libs/urdfdom_headers ] || run_cmd get_urdfdom_headers $prefix/libs
[ -d $prefix/libs/libiconv-1.14 ] || run_cmd get_libiconv $prefix/libs
[ -d $prefix/libs/libxml2-2.9.1 ] || run_cmd get_libxml2 $prefix/libs
[ -d $prefix/libs/collada-dom-2.4.0 ] || run_cmd get_collada_dom $prefix/libs
[ -d $prefix/libs/eigen ] || run_cmd get_eigen $prefix/libs
[ -d $prefix/libs/assimp-3.1.1 ] || run_cmd get_assimp $prefix/libs
[ -d $prefix/libs/qhull-2012.1 ] || run_cmd get_qhull $prefix/libs
[ -d $prefix/libs/octomap-1.6.8 ] || run_cmd get_octomap $prefix/libs

[ -f $prefix/target/bin/catkin_make ] || run_cmd build_catkin $prefix/libs/catkin
. $prefix/target/setup.bash


# Remove catkin package information from target since we will be cross-compiling it,
# otherwise catkin will detect it as duplicate
rm -rf $prefix/target/share/*

echo
echo -e '\e[34mGetting ROS packages\e[39m'
echo

if [[ $skip -ne 1 ]] ; then
    run_cmd get_ros_stuff $prefix/libs

    echo
    echo -e '\e[34mApplying patches.\e[39m'
    echo

    # patch CMakeLists.txt for lz4 library - Build as a library
    patch -p0 -N -d $prefix < /opt/roscpp_android/patches/lz4.patch

    # Patch collada - Build as static lib
    patch -p0 -N -d $prefix < /opt/roscpp_android/patches/collada_dom.patch

    #  Patch assimp - Build as static lib
    patch -p0 -N -d $prefix < /opt/roscpp_android/patches/assimp.patch

    # Patch urdfdom - Build as static lib
    patch -p0 -N -d $prefix < /opt/roscpp_android/patches/urdfdom.patch

    # Patch libiconv - Remove 'gets' error
    patch -p0 -N -d $prefix < /opt/roscpp_android/patches/libiconv.patch

    ## ROS patches

    # Patch roscpp - avoid using ifaddrs on Android as it is not natively supported
    # (TODO: remove once https://github.com/ros/ros_comm/pull/518 is accepted)
    patch -p0 -N -d $prefix < /opt/roscpp_android/patches/roscpp.patch

    # Patch roslz4 - remove python stuff
    # TODO: remove once https://github.com/ros/ros_comm/pull/521 is accepted
    patch -p0 -N -d $prefix < /opt/roscpp_android/patches/roslz4.patch

    # Patch dynamic_reconfigure - Create static lib
    # https://github.com/ros/dynamic_reconfigure/pull/42 merged, need to wait until new version
    patch -p0 -N -d $prefix < /opt/roscpp_android/patches/dynamic_reconfigure.patch

    # Patch class_loader - Create static lib
    # https://github.com/ros/class_loader/pull/20 merged, need to wait until new version
    patch -p0 -N -d $prefix < /opt/roscpp_android/patches/class_loader.patch

    # Patch roslib - weird issue with rospack. TODO: Need to look further (only on catkin_make_isolated)
    #patch -p0 -N -d $prefix < /opt/roscpp_android/patches/roslib.patch

    # Patch collada_parser - cmake detects mkstemps even though Android does not support it
    # TODO: investigate how to prevent cmake to detect system mkstemps
    patch -p0 -N -d $prefix < /opt/roscpp_android/patches/collada_parser.patch

    # Patch urdf - Add ARCHIVE DESTINATION
    # TODO: remove once https://github.com/ros/robot_model/pull/91 is accepted
    patch -p0 -N -d $prefix < /opt/roscpp_android/patches/urdf.patch

fi

echo
echo -e '\e[34mBuilding library dependencies.\e[39m'
echo

[ -f $prefix/target/lib/libbz2.a ] || run_cmd build_bzip2 $prefix/libs/bzip2
[ -f $prefix/target/lib/libuuid.a ] || run_cmd build_uuid $prefix/libs/uuid
[ -f $prefix/target/lib/libboost_system.a ] || run_cmd copy_boost $prefix/libs/boost
[ -f $prefix/target/lib/libPocoFoundation.a ] || run_cmd build_poco $prefix/libs/poco-1.4.6p2
[ -f $prefix/target/lib/libtinyxml.a ] || run_cmd build_tinyxml $prefix/libs/tinyxml
[ -f $prefix/target/lib/libconsole_bridge.a ] || run_cmd build_console_bridge $prefix/libs/console_bridge
[ -f $prefix/target/lib/liblz4.a ] || run_cmd build_lz4 $prefix/libs/lz4-r124/cmake_unofficial
[ -f $prefix/target/lib/libcurl.a ] || run_cmd build_curl $prefix/libs/curl-7.39.0
[ -f $prefix/target/lib/share/urdfdom_headers/cmake/urdfdom_headers-config.cmake ] || run_cmd build_urdfdom_headers $prefix/libs/urdfdom_headers
[ -f $prefix/target/lib/liburdfdom.a ] || run_cmd build_urdfdom $prefix/libs/urdfdom
[ -f $prefix/target/lib/libiconv.a ] || run_cmd build_libiconv $prefix/libs/libiconv-1.14
[ -f $prefix/target/lib/libxml2.a ] || run_cmd build_libxml2 $prefix/libs/libxml2-2.9.1
[ -f $prefix/target/lib/libcollada-dom2.4-dp.a ] || run_cmd build_collada_dom $prefix/libs/collada-dom-2.4.0
[ -f $prefix/target/lib/libassimp.a ] || run_cmd build_assimp $prefix/libs/assimp-3.1.1
[ -f $prefix/target/lib/libeigen.a ] || run_cmd build_eigen $prefix/libs/eigen
[ -f $prefix/target/lib/libqhullstatic.a ] || run_cmd build_qhull $prefix/libs/qhull-2012.1
[ -f $prefix/target/share/octomap/octomap-config.cmake ] || run_cmd build_octomap $prefix/libs/octomap-1.6.8

echo
echo -e '\e[34mCross-compiling ROS.\e[39m'
echo

if [[ $debugging -eq 1 ]];then
    run_cmd build_cpp --debug-symbols
else
    run_cmd build_cpp
fi

echo
echo -e '\e[34mSetting up ndk project.\e[39m'
echo

run_cmd setup_ndk_project $prefix/roscpp_android_ndk

echo
echo -e '\e[34mCreating Android.mk.\e[39m'
echo

# Library path is incorrect for urdf.
# TODO: Need to investigate the source of the issue
sed -i 's/set(libraries "urdf;/set(libraries "/g' $CMAKE_PREFIX_PATH/share/urdf/cmake/urdfConfig.cmake

run_cmd create_android_mk $prefix/target/catkin_ws/src $prefix/roscpp_android_ndk

# JAC: Disabled temporarily and replaced by application-specific Android.mk since
# the library order resulting from create_android_mk doesn't work
#cp $my_loc/files/Android.mk.algron $prefix/roscpp_android_ndk/Android.mk

if [[ $debugging -eq 1 ]];then
    sed -i "s/#LOCAL_EXPORT_CFLAGS/LOCAL_EXPORT_CFLAGS/g" $prefix/roscpp_android_ndk/Android.mk
fi

echo
echo -e '\e[34mCreating sample app.\e[39m'
echo

( cd $prefix && run_cmd sample_app sample_app $prefix/roscpp_android_ndk )

echo
echo -e '\e[34mBuilding apk.\e[39m'
echo

(cd $prefix/sample_app && ant debug)

echo
echo 'done.'
echo 'summary of what just happened:'
echo '  target/      was used to build static libraries for ros software'
echo '    include/   contains headers'
echo '    lib/       contains static libraries'
echo '  roscpp_android_ndk/     is a NDK sub-project that can be imported into an NDK app'
echo '  sample_app/  is an example of such an app, a native activity'
echo '  sample_app/bin/sample_app-debug.apk  is the built apk'
