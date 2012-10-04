# common source files for both node and nodelet driver interfaces
set(DRIVER_SOURCES 
  driver_matrix_vision_camera.cpp
  dev_matrix_vision_camera.cpp
  features.cpp
  formats.cpp
  )

rosbuild_add_library(matrix_vision_camera_driver ${DRIVER_SOURCES})
target_link_libraries(matrix_vision_camera_driver mvDeviceManager)

rosbuild_add_executable(matrix_vision_camera_node matrix_vision_camera_node.cpp )
target_link_libraries(matrix_vision_camera_node  matrix_vision_camera_driver)

#rosbuild_add_library(matrix_vision_camera_nodelet nodelet.cpp)
#target_link_libraries(matrix_vision_camera_nodelet matrix_vision_camera_driver)

#rosbuild_add_executable(matrix_vision_camera_node node.cpp)
