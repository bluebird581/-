project(perception_ipm_test)
cmake_minimum_required(VERSION 2.8)
IF(CMAKE_VERSION VERSION_GREATER "2.8.11")
  CMAKE_POLICY(SET CMP0022 OLD)
ENDIF()
message('project source dir' ${PROJECT_SOURCE_DIR})

add_definitions(-std=c++11)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(/usr/local/cuda-8.0/targets/x86_64-linux/include/)
include_directories(/usr/local/cuda-8.0/targets/aarch64-linux/include/)

find_package(OpenCV)

message("CMAKE_SOURCE_DIR is -----------------" ${CMAKE_SOURCE_DIR})

set(Caffe_DIR ${CMAKE_SOURCE_DIR}/thirdparty/caffe-deeplab/)
include_directories(${Caffe_DIR}/include/)
include_directories(${Caffe_DIR}/build/src/)

add_library(perception_ipm SHARED src/scene_interface.cpp src/camera_localization.cc)
target_link_libraries(perception_ipm 
                      jsoncpp
                      pthread
                      dl
                      ${OpenCV_LIBS}
                      ${Caffe_DIR}/.build_release/lib/libcaffe.so)

add_executable(perception_ipm_test src/localization_test.cc)
target_link_libraries(perception_ipm_test 
                      ${PROJECT_SOURCE_DIR}/build/libperception_ipm.so
                      jsoncpp 
                      ${OpenCV_LIBS} 
                      pthread 
                      dl 
                      matio 
                      protobuf 
                      ${Caffe_DIR}/.build_release/lib/libcaffe.so
                     )
                    
