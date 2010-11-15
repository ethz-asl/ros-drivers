cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/CMakeModules/")

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

find_package(USB REQUIRED)

find_path(LIBFREENECT_INCLUDE_DIR libfreenect.h
	/usr/local/include
	/usr/include
)
find_library(LIBFREENECT_LIBRARY freenect
	/usr/local/lib
	/usr/lib
)

include_directories(${USB_INCLUDE_DIRS} ${LIBFREENECT_INCLUDE_DIR})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_boost_directories()
rosbuild_add_executable(${PROJECT_NAME} src/kinect.cpp)
rosbuild_link_boost(${PROJECT_NAME} thread)
target_link_libraries(${PROJECT_NAME} ${LIBFREENECT_LIBRARY} ${USB_LIBRARIES})
