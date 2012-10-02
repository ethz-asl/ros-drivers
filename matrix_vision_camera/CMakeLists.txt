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

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# detect platform
if(CMAKE_SIZEOF_VOID_P MATCHES "8")
#    set(MVIMPACT_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/mvIMPACT_SDK/mvBlueCOUGAR-x86_64_ABI2-2.0.4/mvIMPACT_acquire-2.0.4)
#    set(MVIMPACT_LINK_DIR    ${PROJECT_SOURCE_DIR}/mvIMPACT_SDK/mvIMPACT_acquire-x86_64-2.0.4/lib/x86_64)    
    set(MVIMPACT_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/matrix_vision_drivers/x86_64/mvIMPACT_acquire-2.0.5)
    set(MVIMPACT_LINK_DIR  
        ${PROJECT_SOURCE_DIR}/matrix_vision_drivers/x86_64/mvIMPACT_acquire-2.0.5/lib/x86_64
        ${PROJECT_SOURCE_DIR}/matrix_vision_drivers/x86_64/mvBlueFOX_runtime-2.0.5
        ${PROJECT_SOURCE_DIR}/matrix_vision_drivers/x86_64/mvBlueCOUGAR_runtime-2.0.5/lib
        ${PROJECT_SOURCE_DIR}/matrix_vision_drivers/x86_64/mvBlueCOUGAR_runtime-2.0.5/GenICam_Runtime_gcc40_Linux64_x64_v2_3_0/bin/Linux64_x64
        ${PROJECT_SOURCE_DIR}/matrix_vision_drivers/x86_64/mvBlueCOUGAR_runtime-2.0.5/GenICam_Runtime_gcc40_Linux64_x64_v2_3_0/bin/Linux64_x64/GenApi/Generic
        ) 
    set(GENICAM_ROOT ${PROJECT_SOURCE_DIR}/matrix_vision_drivers/x86_64/mvBlueCOUGAR_runtime-2.0.5/GenICam_Runtime_gcc40_Linux64_x64_v2_3_0) 
    set(GENICAM_ROOT_V2_3 ${PROJECT_SOURCE_DIR}/matrix_vision_drivers/x86_64/mvBlueCOUGAR_runtime-2.0.5/GenICam_Runtime_gcc40_Linux64_x64_v2_3_0) 
    set(GENICAM_GENTL64_PATH ${PROJECT_SOURCE_DIR}/matrix_vision_drivers/x86_64/mvBlueCOUGAR_runtime-2.0.5/lib)
    set(GENICAM_CACHE_V2_3 ${PROJECT_SOURCE_DIR}matrix_vision_drivers/genicam_cache)
    set(GENICAM_LOG_CONFIG_V2_3 ${PROJECT_SOURCE_DIR}/matrix_vision_drivers/x86_64/mvBlueCOUGAR_runtime-2.0.5/GenICam_Runtime_gcc40_Linux64_x64_v2_3_0/log/config-unix/DefaultLogging.properties)
    
elseif(CMAKE_SIZEOF_VOID_P MATCHES "4")
    set(MVIMPACT_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/matrix_vision_drivers/x86_32/mvIMPACT_acquire)
    set(MVIMPACT_LINK_DIR  
        ${PROJECT_SOURCE_DIR}/matrix_vision_drivers/x86/mvIMPACT_acquire/lib/x86
        ${PROJECT_SOURCE_DIR}/matrix_vision_drivers/x86/mvBlueFOX_runtime
        ${PROJECT_SOURCE_DIR}/matrix_vision_drivers/x86/mvBlueCOUGAR_runtime/lib
        ${PROJECT_SOURCE_DIR}/matrix_vision_drivers/x86/mvBlueCOUGAR_runtime/GenICam_Runtime_gcc40_Linux32_i86_v2_3_0/bin/Linux32_i86
        ${PROJECT_SOURCE_DIR}/matrix_vision_drivers/x86/mvBlueCOUGAR_runtime/GenICam_Runtime_gcc40_Linux32_i86_v2_3_0/bin/Linux32_i86/GenApi/Generic
        ) 
    set(GENICAM_ROOT ${PROJECT_SOURCE_DIR}/matrix_vision_drivers/x86/mvBlueCOUGAR_runtime/GenICam_Runtime_gcc40_Linux32_i86_v2_3_0) 
    set(GENICAM_ROOT_V2_3 ${PROJECT_SOURCE_DIR}/matrix_vision_drivers/x86/mvBlueCOUGAR_runtime/GenICam_Runtime_gcc40_Linux32_i86_v2_3_0) 
    set(GENICAM_GENTL64_PATH ${PROJECT_SOURCE_DIR}/matrix_vision_drivers/x86/mvBlueCOUGAR_runtime/lib)
    set(GENICAM_CACHE_V2_3 ${PROJECT_SOURCE_DIR}matrix_vision_drivers/genicam_cache)
    set(GENICAM_LOG_CONFIG_V2_3 ${PROJECT_SOURCE_DIR}/matrix_vision_drivers/x86/mvBlueCOUGAR_runtime/GenICam_Runtime_gcc40_Linux32_i86_v2_3_0/log/config-unix/DefaultLogging.properties)




else(CMAKE_SIZEOF_VOID_P MATCHES "8")
    message(FATAL_ERROR "unable to detect platform")
endif (CMAKE_SIZEOF_VOID_P MATCHES "8")

include_directories(${INCLUDE_DIRECTORIES} ${MVIMPACT_INCLUDE_DIR})
link_directories(${LINK_DIRECTORIES} ${MVIMPACT_LINK_DIR})

add_definitions(
    -DGENICAM_ROOT='"${GENICAM_ROOT}"'
    -DGENICAM_ROOT_V2_3='"${GENICAM_ROOT_V2_3}"'
    -DGENICAM_GENTL64_PATH='"${GENICAM_GENTL64_PATH}"'
    -DGENICAM_CACHE_V2_3='"${GENICAM_CACHE_V2_3}"'
    -DGENICAM_LOG_CONFIG_V2_3='"${GENICAM_LOG_CONFIG_V2_3}"'    
    )

# auto-generated dynamic reconfiguration GUI
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)

gencfg()



#rosbuild_add_executable(livesnap src/LiveSnap.cpp)

rosbuild_add_executable(driverInterface src/driverInterface)


add_subdirectory(${PROJECT_SOURCE_DIR}/src/nodes)
