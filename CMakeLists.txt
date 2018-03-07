cmake_minimum_required(VERSION 2.8.3)
project(openpose_ros)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  cv_bridge
  image_transport
  gflags
  glog_catkin
  openpose_ros_msgs
)


find_package(CUDA REQUIRED)
find_package(OpenCV REQUIRED)

set(CAFFE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/openpose/3rdparty/caffe)
set(OPENPOSE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/openpose)

set(CAFFE_INCLUDE_DIRS ${CAFFE_DIR}/distribute/include)
set(CAFFE_LINK_LIBRARY ${CAFFE_DIR}/distribute/lib/libcaffe.so)

set(OPENPOSE_INCLUDE_DIRS ${OPENPOSE_DIR}/include)
set(OPENPOSE_LINK_LIBRARY ${OPENPOSE_DIR}/build/lib/libopenpose.so)

catkin_package(
 CATKIN_DEPENDS
 roscpp
 rospy
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${CAFFE_INCLUDE_DIRS}
  ${OPENPOSE_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/openpose_ros_node.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${CAFFE_LINK_LIBRARY}
  ${OPENPOSE_LINK_LIBRARY}
)


