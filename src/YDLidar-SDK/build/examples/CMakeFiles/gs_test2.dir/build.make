# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/ubuntu/f1tenth_ws/src/YDLidar-SDK

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ubuntu/f1tenth_ws/src/YDLidar-SDK/build

# Include any dependencies generated for this target.
include examples/CMakeFiles/gs_test2.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include examples/CMakeFiles/gs_test2.dir/compiler_depend.make

# Include the progress variables for this target.
include examples/CMakeFiles/gs_test2.dir/progress.make

# Include the compile flags for this target's objects.
include examples/CMakeFiles/gs_test2.dir/flags.make

examples/CMakeFiles/gs_test2.dir/gs_test2.cpp.o: examples/CMakeFiles/gs_test2.dir/flags.make
examples/CMakeFiles/gs_test2.dir/gs_test2.cpp.o: ../examples/gs_test2.cpp
examples/CMakeFiles/gs_test2.dir/gs_test2.cpp.o: examples/CMakeFiles/gs_test2.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/f1tenth_ws/src/YDLidar-SDK/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/CMakeFiles/gs_test2.dir/gs_test2.cpp.o"
	cd /home/ubuntu/f1tenth_ws/src/YDLidar-SDK/build/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT examples/CMakeFiles/gs_test2.dir/gs_test2.cpp.o -MF CMakeFiles/gs_test2.dir/gs_test2.cpp.o.d -o CMakeFiles/gs_test2.dir/gs_test2.cpp.o -c /home/ubuntu/f1tenth_ws/src/YDLidar-SDK/examples/gs_test2.cpp

examples/CMakeFiles/gs_test2.dir/gs_test2.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gs_test2.dir/gs_test2.cpp.i"
	cd /home/ubuntu/f1tenth_ws/src/YDLidar-SDK/build/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/f1tenth_ws/src/YDLidar-SDK/examples/gs_test2.cpp > CMakeFiles/gs_test2.dir/gs_test2.cpp.i

examples/CMakeFiles/gs_test2.dir/gs_test2.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gs_test2.dir/gs_test2.cpp.s"
	cd /home/ubuntu/f1tenth_ws/src/YDLidar-SDK/build/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/f1tenth_ws/src/YDLidar-SDK/examples/gs_test2.cpp -o CMakeFiles/gs_test2.dir/gs_test2.cpp.s

# Object files for target gs_test2
gs_test2_OBJECTS = \
"CMakeFiles/gs_test2.dir/gs_test2.cpp.o"

# External object files for target gs_test2
gs_test2_EXTERNAL_OBJECTS =

gs_test2: examples/CMakeFiles/gs_test2.dir/gs_test2.cpp.o
gs_test2: examples/CMakeFiles/gs_test2.dir/build.make
gs_test2: libydlidar_sdk.a
gs_test2: examples/CMakeFiles/gs_test2.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ubuntu/f1tenth_ws/src/YDLidar-SDK/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../gs_test2"
	cd /home/ubuntu/f1tenth_ws/src/YDLidar-SDK/build/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/gs_test2.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/CMakeFiles/gs_test2.dir/build: gs_test2
.PHONY : examples/CMakeFiles/gs_test2.dir/build

examples/CMakeFiles/gs_test2.dir/clean:
	cd /home/ubuntu/f1tenth_ws/src/YDLidar-SDK/build/examples && $(CMAKE_COMMAND) -P CMakeFiles/gs_test2.dir/cmake_clean.cmake
.PHONY : examples/CMakeFiles/gs_test2.dir/clean

examples/CMakeFiles/gs_test2.dir/depend:
	cd /home/ubuntu/f1tenth_ws/src/YDLidar-SDK/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ubuntu/f1tenth_ws/src/YDLidar-SDK /home/ubuntu/f1tenth_ws/src/YDLidar-SDK/examples /home/ubuntu/f1tenth_ws/src/YDLidar-SDK/build /home/ubuntu/f1tenth_ws/src/YDLidar-SDK/build/examples /home/ubuntu/f1tenth_ws/src/YDLidar-SDK/build/examples/CMakeFiles/gs_test2.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/CMakeFiles/gs_test2.dir/depend
