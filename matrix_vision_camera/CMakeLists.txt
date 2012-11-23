cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set(ROS_BUILD_TYPE Debug)

rosbuild_gensrv()
#rosbuild_genmsg()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_definitions(-DGENICAM_CACHE_V2_3='"${PROJECT_SOURCE_DIR}/genicam_cache"')

# auto-generated dynamic reconfiguration GUI
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)

gencfg()



#rosbuild_add_executable(livesnap src/LiveSnap.cpp)

rosbuild_add_executable(driverInterface src/driverInterface)


add_subdirectory(${PROJECT_SOURCE_DIR}/src/nodes)
