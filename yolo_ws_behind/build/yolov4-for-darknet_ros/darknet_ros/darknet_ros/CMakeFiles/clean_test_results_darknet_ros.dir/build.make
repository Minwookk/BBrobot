# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/patrol2/yolo_ws_behind/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/patrol2/yolo_ws_behind/build

# Utility rule file for clean_test_results_darknet_ros.

# Include the progress variables for this target.
include yolov4-for-darknet_ros/darknet_ros/darknet_ros/CMakeFiles/clean_test_results_darknet_ros.dir/progress.make

yolov4-for-darknet_ros/darknet_ros/darknet_ros/CMakeFiles/clean_test_results_darknet_ros:
	cd /home/patrol2/yolo_ws_behind/build/yolov4-for-darknet_ros/darknet_ros/darknet_ros && /usr/bin/python2 /opt/ros/melodic/share/catkin/cmake/test/remove_test_results.py /home/patrol2/yolo_ws_behind/build/test_results/darknet_ros

clean_test_results_darknet_ros: yolov4-for-darknet_ros/darknet_ros/darknet_ros/CMakeFiles/clean_test_results_darknet_ros
clean_test_results_darknet_ros: yolov4-for-darknet_ros/darknet_ros/darknet_ros/CMakeFiles/clean_test_results_darknet_ros.dir/build.make

.PHONY : clean_test_results_darknet_ros

# Rule to build all files generated by this target.
yolov4-for-darknet_ros/darknet_ros/darknet_ros/CMakeFiles/clean_test_results_darknet_ros.dir/build: clean_test_results_darknet_ros

.PHONY : yolov4-for-darknet_ros/darknet_ros/darknet_ros/CMakeFiles/clean_test_results_darknet_ros.dir/build

yolov4-for-darknet_ros/darknet_ros/darknet_ros/CMakeFiles/clean_test_results_darknet_ros.dir/clean:
	cd /home/patrol2/yolo_ws_behind/build/yolov4-for-darknet_ros/darknet_ros/darknet_ros && $(CMAKE_COMMAND) -P CMakeFiles/clean_test_results_darknet_ros.dir/cmake_clean.cmake
.PHONY : yolov4-for-darknet_ros/darknet_ros/darknet_ros/CMakeFiles/clean_test_results_darknet_ros.dir/clean

yolov4-for-darknet_ros/darknet_ros/darknet_ros/CMakeFiles/clean_test_results_darknet_ros.dir/depend:
	cd /home/patrol2/yolo_ws_behind/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/patrol2/yolo_ws_behind/src /home/patrol2/yolo_ws_behind/src/yolov4-for-darknet_ros/darknet_ros/darknet_ros /home/patrol2/yolo_ws_behind/build /home/patrol2/yolo_ws_behind/build/yolov4-for-darknet_ros/darknet_ros/darknet_ros /home/patrol2/yolo_ws_behind/build/yolov4-for-darknet_ros/darknet_ros/darknet_ros/CMakeFiles/clean_test_results_darknet_ros.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : yolov4-for-darknet_ros/darknet_ros/darknet_ros/CMakeFiles/clean_test_results_darknet_ros.dir/depend

