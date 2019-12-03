cmake_minimum_required(VERSION 2.8.3)
project(calibr_accelerometer)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3)
include_directories(
  ${EIGEN3_INCLUDE_DIR}
)
find_package(Ceres REQUIRED)

include_directories()

add_executable(calibr_acc src/main.cpp
    src/anker/readAnkerDataFile.cpp
    src/calibrAcc/calibrAcc.cpp
    src/calibrAcc/gravityNormFactor.cpp)

target_link_libraries(calibr_acc ${CERES_LIBRARIES})
