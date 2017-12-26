project(Extended-Kalman-Filter)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(CMAKE_BUILD_TYPE Debug)

include_directories(
  include
  include/EKF
  external_libs)

FILE(GLOB_RECURSE HEADER_FILES include *.h)


add_library(kf 
  src/EKF/kalman_filter.cpp)

add_library(ekf
  src/EKF/extended_kalman_filter.cpp)

add_library(ekf_fusion
  src/ekf_fusion.cpp
  src/ekf_lidar.cpp
  src/ekf_radar.cpp
  src/tools.cpp)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

add_executable(ExtentedKF src/main.cpp ${HEADER_FILES})

target_link_libraries(ekf kf)

target_link_libraries(ekf_fusion ekf)

target_link_libraries(ExtentedKF ekf_fusion z ssl uv uWS)

################################
# GTest
################################
ADD_SUBDIRECTORY (external_libs/gtest-1.8.0)
enable_testing()
include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

################################
# Unit Tests
################################
add_executable(kalman_filter_test test/kalman_filter.cpp)
target_link_libraries(kalman_filter_test kf gtest gtest_main pthread)
add_test(kalman_filter_test kalman_filter_test)

add_executable(ekf_lidar_test test/ekf_lidar.cpp)
target_link_libraries(ekf_lidar_test ekf_fusion gtest gtest_main pthread)
add_test(ekf_lidar_test ekf_lidar_test)
