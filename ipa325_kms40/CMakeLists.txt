cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Scanning the folders to include hpp and h files to qtcreator
file(GLOB_RECURSE MYACTIONS RELATIVE ${PROJECT_SOURCE_DIR} FOLLOW_SYMLINKS action/*.action)

file(GLOB_RECURSE MYMSGS RELATIVE ${PROJECT_SOURCE_DIR} FOLLOW_SYMLINKS msg/*.msg)

# Scanning the folders to include xml files to qtcreator
file(GLOB_RECURSE MYXML RELATIVE ${PROJECT_SOURCE_DIR} FOLLOW_SYMLINKS common/*.xml)

# Scanning the folders to include yaml files to qtcreator
file(GLOB_RECURSE MYYAML RELATIVE ${PROJECT_SOURCE_DIR} FOLLOW_SYMLINKS *.yaml)

# Scanning the folders to include launch files to qtcreator
file(GLOB_RECURSE MYLAUNCH RELATIVE ${PROJECT_SOURCE_DIR} FOLLOW_SYMLINKS *.launch)

#Adding boost for action_lib
rosbuild_add_boost_directories()
#Building RosExecutable
rosbuild_add_executable(Kms40Driver src/Kms40Driver.cpp ${MYHEADERS} ${MYACTIONS} ${MYXML} ${MYLAUNCH} ${MYYAML} ${MYMSGS})

rosbuild_add_executable(MarkerPublisher src/MarkerPublisher.cpp ${MYHEADERS} ${MYACTIONS} ${MYXML} ${MYLAUNCH} ${MYYAML} ${MYMSGS})

#adding boost linking
rosbuild_link_boost(Kms40Driver iostreams system)

#rosbuild_link_boost(MarkerPublisher thread)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
