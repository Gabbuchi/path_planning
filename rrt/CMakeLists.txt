cmake_minimum_required(VERSION 3.10)
project(planning_algorithms)


find_package(OpenCV 4.5.2 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

add_executable(rrt src/rrt.cc)
target_link_libraries(rrt ${OpenCV_LIBS})
