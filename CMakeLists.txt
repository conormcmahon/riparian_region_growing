cmake_minimum_required(VERSION 2.8.3)

project(riparian_region_growing)
#include_directories(include "/usr/include")

# Point Cloud Library (PCL)
find_package(PCL 1.09 REQUIRED COMPONENTS)
include_directories(include ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Computational Geometry Algorithms Library (CGAL)
set(CMAKE_BUILD_TYPE "Release")
find_package(CGAL QUIET COMPONENTS)
if(NOT CGAL_FOUND)
  message(STATUS "This project requires CGAL and will not be compiled.")
  return()
endif()

# Boost and its components
find_package( Boost REQUIRED )
if ( NOT Boost_FOUND )
  message(STATUS "This project requires the Boost library, and will not be compiled.")
  return()
endif()

# GDAL
find_package(GDAL REQUIRED)

add_library(riparian_grower src/riparian_grower.cpp)
target_link_libraries(riparian_grower ${PCL_LIBRARIES} gdal ${CGAL_LIBRARIES})

add_executable(growing_test src/growing_test.cpp)
target_link_libraries(growing_test ${PCL_LIBRARIES} gdal ${CGAL_LIBRARIES})


# Install 
install(TARGETS riparian_grower
        DESTINATION lib)
install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION include
        FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")