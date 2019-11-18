cmake_minimum_required(VERSION 2.8)
project(kitti2map)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_STANDARD 11)
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARIY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(kitti2map main.cpp)
target_link_libraries(kitti2map
  ${PCL_LIBRARIES}
  )