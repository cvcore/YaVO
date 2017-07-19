cmake_minimum_required(VERSION 3.0)
project(YaVO)

# In case the user does not setup CMAKE_BUILD_TYPE, assume it's RelWithDebInfo
if("${CMAKE_BUILD_TYPE}" STREQUAL "")
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "build type default to RelWithDebInfo, set to Release to improve performance" FORCE)
endif("${CMAKE_BUILD_TYPE}" STREQUAL "")

if(CMAKE_BUILD_TYPE STREQUAL "Release" OR CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
  message(STATUS "building in release mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations -std=c++0x -Wno-inconsistent-missing-override")
endif()

# for clang completer
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake_modules)

# for keg-only install of OpenCV 3 on macOS
find_package(OpenCV 3 REQUIRED)
include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OPENCV_LIBRARY_DIRS})
add_definitions(${OPENCV_DEFINITIONS})

# PCL
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# G2O & Friends
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
find_package(G2O REQUIRED)
include_directories(${G2O_LIBRARY})
find_package(CSparse)
include_directories(${CSPARSE_INCLUDE_DIR})

if(NOT G2O_FOUND)
  message(STATUS "System G2O not found!")
#  message(STATUS "Downloading G2O from source!")
#  project(SUPERBUILD NONE)
#  include(cmake_modules/superbuild.cmake)
#  return()
#
else()
  message(STATUS "G2O found! PATH ${G2O_INCLUDE_DIR}")
endif()

# Spdlog
include_directories(${PROJECT_SOURCE_DIR}/spdlog/include)

# paths
include_directories(${PROJECT_SOURCE_DIR}/include)
#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_library(RGBDSlam
        src/rgbd_camera.cc
        src/rgbd_frame.cc
        src/rgbd_odometer.cc
        src/rgbd_parameter_reader.cc
	src/rgbd_visualizer.cc)

target_link_libraries(RGBDSlam
        ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES} ${PCL_FILTERS_LIBRARIES}
        ${OpenCV_LIBS}
        g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension ${CSPARSE_LIBRARY})

add_executable(RGBDSlamApp src/rgbd_slam_app.cc)
target_link_libraries(RGBDSlamApp RGBDSlam)
