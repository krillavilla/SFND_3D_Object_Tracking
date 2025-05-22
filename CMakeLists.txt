cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(camera_fusion)

find_package(OpenCV 4.1 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

# Check if OpenCV xfeatures2d module is available
if(NOT "${OpenCV_LIBS}" MATCHES "opencv_xfeatures2d")
    message(STATUS "OpenCV xfeatures2d module not found. SIFT and SURF will be disabled.")
else()
    message(STATUS "OpenCV xfeatures2d module found. SIFT and SURF will be enabled.")
    add_definitions(-DOPENCV_ENABLE_NONFREE)
endif()

# Executable for create matrix exercise
add_executable (3D_object_tracking src/camFusion_Student.cpp src/FinalProject_Camera.cpp src/lidarData.cpp src/matching2D_Student.cpp src/objectDetection2D.cpp)
target_link_libraries (3D_object_tracking ${OpenCV_LIBRARIES})