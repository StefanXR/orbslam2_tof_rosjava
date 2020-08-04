cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(pcl_visualizer_viewports)

# Check for c++14 support
INCLUDE(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX11)
IF(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS}")
ELSE()
  MESSAGE(ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support. Please use a different C++ compiler.")
ENDIF()

find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (gicp gicp.cpp)
target_link_libraries (gicp ${PCL_LIBRARIES})
add_executable (viewer viewer.cpp)
target_link_libraries (viewer ${PCL_LIBRARIES})
