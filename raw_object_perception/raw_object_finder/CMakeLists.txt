cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

find_package(wxWidgets REQUIRED)
include(${wxWidgets_USE_FILE})
#include_directories(${wxWidgets_INCLUDE})

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()
######################OpenMP
set(CCMAKE_CXX_FLAGS -fopenmp)
set(CCMAKE_C_FLAGS -fopenmp)
set(CMAKE_EXE_LINKER_FLAGS -lgomp)
#####################OCS
include_directories(
${PROJECT_SOURCE_DIR}/ros/include
/usr/local/include/opencv
/usr/include/opencv
${wxWidgets_INCLUDE}
)

link_directories(
/usr/local/lib
/usr/lib
)

link_libraries(
opencv_highgui
opencv_core
)


#########################


###Object Candidate extraction
rosbuild_add_library(CToolBoxROS ros/src/CToolBoxROS.cpp)
rosbuild_add_library(CPlaneExtraction ros/src/CPlaneExtraction.cpp)
rosbuild_add_library(CObjectCandidateExtraction ros/src/CObjectCandidateExtraction.cpp)

rosbuild_add_executable(objectCandidateExtraction3D_node ros/src/objectCandidateExtraction3D_node.cpp)
target_link_libraries(objectCandidateExtraction3D_node CToolBoxROS CPlaneExtraction CObjectCandidateExtraction)

#rosbuild_add_executable(3dTo2dProjection_node ros/src/3dTo2dProjection_node.cpp)
#rosbuild_add_executable(objectRecognition_node  ros/src/objectRecognition_node.cpp )
#target_link_libraries(objectRecognition_node CToolBoxROS mds objectGeometricAnalyzer)
