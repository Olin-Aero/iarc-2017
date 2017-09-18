cmake_minimum_required(VERSION 2.8.3)
project(neato_node)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED COMPONENTS message_generation roscpp tf laser_geometry cv_bridge image_transport roscpp camera_info_manager cmake_modules)
find_package(OpenCV)
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN_INCLUDE_DIRS})

add_definitions(${EIGEN_DEFINITIONS})
add_message_files(FILES Bump.msg Accel.msg)
generate_messages(DEPENDENCIES std_msgs)

add_executable(fix_scan_fast src/fix_scan_fast.cpp)
target_link_libraries(fix_scan_fast ${catkin_LIBRARIES})

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
include_directories(${catkin_INCLUDE_DIRS})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY nodes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

