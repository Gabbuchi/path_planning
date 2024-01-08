cmake_minimum_required(VERSION 3.8 FATAL_ERROR)
project(dwa LANGUAGES CXX)

include(GNUInstallDirs)
set(PACKAGE_NAME dwa)
set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/${PACKAGE_NAME}/cmake)

include_directories(${EIGEN3_INCLUDE_DIR})

find_package(yaml-cpp REQUIRED)

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
add_executable(aStar src/main.cc src/a_star.cc)
target_link_libraries(aStar PRIVATE matplotlib_cpp yaml-cpp)
#set_target_properties(dwa PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

# Install headers
install(FILES
  "${PROJECT_SOURCE_DIR}/matplotlibcpp.h"
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# Install targets file
install(EXPORT install_targets
  FILE
    ${PACKAGE_NAME}Targets.cmake
  NAMESPACE
    ${PACKAGE_NAME}::
  DESTINATION
    ${INSTALL_CONFIGDIR}
)


# Install matplotlib_cppConfig.cmake
# include(CMakePackageConfigHelpers)
# configure_package_config_file(
#   ${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PACKAGE_NAME}Config.cmake.in
#   ${CMAKE_CURRENT_BINARY_DIR}/${PACKAGE_NAME}Config.cmake
#   INSTALL_DESTINATION ${INSTALL_CONFIGDIR}
# )
# install(FILES
#   ${CMAKE_CURRENT_BINARY_DIR}/${PACKAGE_NAME}Config.cmake
#   DESTINATION ${INSTALL_CONFIGDIR}
# )
