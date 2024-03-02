cmake_minimum_required(VERSION 3.10)
project(planning_algorithms)


find_package(OpenCV 4.5.2 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
# Library target
add_library(matplotlib_cpp INTERFACE)
target_include_directories(matplotlib_cpp
  INTERFACE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/examples>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(matplotlib_cpp INTERFACE
  cxx_std_11
)
# TODO: Use `Development.Embed` component when requiring cmake >= 3.18
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
target_link_libraries(matplotlib_cpp INTERFACE
  Python3::Python
  Python3::Module
)
find_package(Python3 COMPONENTS NumPy)
if(Python3_NumPy_FOUND)
  target_link_libraries(matplotlib_cpp INTERFACE
    Python3::NumPy
  )
else()
  target_compile_definitions(matplotlib_cpp INTERFACE WITHOUT_NUMPY)
endif()
install(
  TARGETS matplotlib_cpp
  EXPORT install_targets
)


# Examples
add_executable(rrt src/cubic_spline.cpp src/rrt.cc)
target_link_libraries(rrt PRIVATE matplotlib_cpp yaml-cpp ${OpenCV_LIBS})


