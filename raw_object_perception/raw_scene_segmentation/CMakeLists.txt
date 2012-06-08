cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Debug)

rosbuild_init()

#...: compiler options :........................................................

#.... gnu++0x
if(CMAKE_COMPILER_IS_GNUCXX)
    execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
    if(GCC_VERSION VERSION_GREATER 4.4 OR GCC_VERSION VERSION_EQUAL 4.4)
        add_definitions(-std=gnu++0x)
    else(GCC_VERSION VERSION_GREATER 4.4 OR GCC_VERSION VERSION_EQUAL 4.4)
        message(SEND_ERROR "You need GCC version 4.4 or greater to compile this package.")
    endif(GCC_VERSION VERSION_GREATER 4.4 OR GCC_VERSION VERSION_EQUAL 4.4)
endif(CMAKE_COMPILER_IS_GNUCXX)

#.... treat warnings as errors
add_definitions(-Werror)
add_definitions(-Wno-error=unused-variable)
add_definitions(-Wno-error=unknown-pragmas)
add_definitions(-Wno-deprecated)

#...: default paths  :..........................................................

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
include_directories(${PROJECT_SOURCE_DIR}/ros/include)

#...: target libraries :........................................................

rosbuild_add_library(scene_segmentation ros/src/dominant_plane_extractor.cpp
                                        ros/src/organized_dominant_plane_extractor.cpp
                                        ros/src/ransac_dominant_plane_extractor.cpp
                                        ros/src/tabletop_cluster_extractor.cpp
)

#...: target executables :......................................................

#...: dominant_plane_extractor_node
rosbuild_add_executable(dominant_plane_extractor_node ros/src/nodes/dominant_plane_extractor_node.cpp
)
target_link_libraries(dominant_plane_extractor_node scene_segmentation
)
