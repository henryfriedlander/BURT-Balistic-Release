# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/robot/proficio_toolbox/tests

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/robot/proficio_toolbox/tests

# Include any dependencies generated for this target.
include safety_systems/CMakeFiles/virtual_joint_stops_test.dir/depend.make

# Include the progress variables for this target.
include safety_systems/CMakeFiles/virtual_joint_stops_test.dir/progress.make

# Include the compile flags for this target's objects.
include safety_systems/CMakeFiles/virtual_joint_stops_test.dir/flags.make

safety_systems/CMakeFiles/virtual_joint_stops_test.dir/virtual_joint_stops_test.cpp.o: safety_systems/CMakeFiles/virtual_joint_stops_test.dir/flags.make
safety_systems/CMakeFiles/virtual_joint_stops_test.dir/virtual_joint_stops_test.cpp.o: safety_systems/virtual_joint_stops_test.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/robot/proficio_toolbox/tests/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object safety_systems/CMakeFiles/virtual_joint_stops_test.dir/virtual_joint_stops_test.cpp.o"
	cd /home/robot/proficio_toolbox/tests/safety_systems && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/virtual_joint_stops_test.dir/virtual_joint_stops_test.cpp.o -c /home/robot/proficio_toolbox/tests/safety_systems/virtual_joint_stops_test.cpp

safety_systems/CMakeFiles/virtual_joint_stops_test.dir/virtual_joint_stops_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/virtual_joint_stops_test.dir/virtual_joint_stops_test.cpp.i"
	cd /home/robot/proficio_toolbox/tests/safety_systems && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/robot/proficio_toolbox/tests/safety_systems/virtual_joint_stops_test.cpp > CMakeFiles/virtual_joint_stops_test.dir/virtual_joint_stops_test.cpp.i

safety_systems/CMakeFiles/virtual_joint_stops_test.dir/virtual_joint_stops_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/virtual_joint_stops_test.dir/virtual_joint_stops_test.cpp.s"
	cd /home/robot/proficio_toolbox/tests/safety_systems && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/robot/proficio_toolbox/tests/safety_systems/virtual_joint_stops_test.cpp -o CMakeFiles/virtual_joint_stops_test.dir/virtual_joint_stops_test.cpp.s

safety_systems/CMakeFiles/virtual_joint_stops_test.dir/virtual_joint_stops_test.cpp.o.requires:
.PHONY : safety_systems/CMakeFiles/virtual_joint_stops_test.dir/virtual_joint_stops_test.cpp.o.requires

safety_systems/CMakeFiles/virtual_joint_stops_test.dir/virtual_joint_stops_test.cpp.o.provides: safety_systems/CMakeFiles/virtual_joint_stops_test.dir/virtual_joint_stops_test.cpp.o.requires
	$(MAKE) -f safety_systems/CMakeFiles/virtual_joint_stops_test.dir/build.make safety_systems/CMakeFiles/virtual_joint_stops_test.dir/virtual_joint_stops_test.cpp.o.provides.build
.PHONY : safety_systems/CMakeFiles/virtual_joint_stops_test.dir/virtual_joint_stops_test.cpp.o.provides

safety_systems/CMakeFiles/virtual_joint_stops_test.dir/virtual_joint_stops_test.cpp.o.provides.build: safety_systems/CMakeFiles/virtual_joint_stops_test.dir/virtual_joint_stops_test.cpp.o

# Object files for target virtual_joint_stops_test
virtual_joint_stops_test_OBJECTS = \
"CMakeFiles/virtual_joint_stops_test.dir/virtual_joint_stops_test.cpp.o"

# External object files for target virtual_joint_stops_test
virtual_joint_stops_test_EXTERNAL_OBJECTS =

safety_systems/virtual_joint_stops_test: safety_systems/CMakeFiles/virtual_joint_stops_test.dir/virtual_joint_stops_test.cpp.o
safety_systems/virtual_joint_stops_test: /usr/lib/libCGAL_Core.so
safety_systems/virtual_joint_stops_test: /usr/lib/libCGAL.so
safety_systems/virtual_joint_stops_test: /usr/lib/x86_64-linux-gnu/libgmpxx.so
safety_systems/virtual_joint_stops_test: /usr/lib/x86_64-linux-gnu/libmpfr.so
safety_systems/virtual_joint_stops_test: /usr/lib/x86_64-linux-gnu/libgmp.so
safety_systems/virtual_joint_stops_test: /usr/lib/libboost_thread-mt.so
safety_systems/virtual_joint_stops_test: /usr/lib/libboost_system-mt.so
safety_systems/virtual_joint_stops_test: /usr/lib/libboost_thread-mt.so
safety_systems/virtual_joint_stops_test: /usr/lib/libboost_python.so
safety_systems/virtual_joint_stops_test: /usr/lib/libnative.so
safety_systems/virtual_joint_stops_test: /usr/lib/libxenomai.so
safety_systems/virtual_joint_stops_test: /usr/lib/librtdm.so
safety_systems/virtual_joint_stops_test: /usr/lib/libpython2.7.so
safety_systems/virtual_joint_stops_test: /usr/lib/libboost_system-mt.so
safety_systems/virtual_joint_stops_test: /usr/lib/libboost_python.so
safety_systems/virtual_joint_stops_test: /usr/lib/libnative.so
safety_systems/virtual_joint_stops_test: /usr/lib/libxenomai.so
safety_systems/virtual_joint_stops_test: /usr/lib/librtdm.so
safety_systems/virtual_joint_stops_test: /usr/lib/libpython2.7.so
safety_systems/virtual_joint_stops_test: safety_systems/CMakeFiles/virtual_joint_stops_test.dir/build.make
safety_systems/virtual_joint_stops_test: safety_systems/CMakeFiles/virtual_joint_stops_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable virtual_joint_stops_test"
	cd /home/robot/proficio_toolbox/tests/safety_systems && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/virtual_joint_stops_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
safety_systems/CMakeFiles/virtual_joint_stops_test.dir/build: safety_systems/virtual_joint_stops_test
.PHONY : safety_systems/CMakeFiles/virtual_joint_stops_test.dir/build

safety_systems/CMakeFiles/virtual_joint_stops_test.dir/requires: safety_systems/CMakeFiles/virtual_joint_stops_test.dir/virtual_joint_stops_test.cpp.o.requires
.PHONY : safety_systems/CMakeFiles/virtual_joint_stops_test.dir/requires

safety_systems/CMakeFiles/virtual_joint_stops_test.dir/clean:
	cd /home/robot/proficio_toolbox/tests/safety_systems && $(CMAKE_COMMAND) -P CMakeFiles/virtual_joint_stops_test.dir/cmake_clean.cmake
.PHONY : safety_systems/CMakeFiles/virtual_joint_stops_test.dir/clean

safety_systems/CMakeFiles/virtual_joint_stops_test.dir/depend:
	cd /home/robot/proficio_toolbox/tests && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/robot/proficio_toolbox/tests /home/robot/proficio_toolbox/tests/safety_systems /home/robot/proficio_toolbox/tests /home/robot/proficio_toolbox/tests/safety_systems /home/robot/proficio_toolbox/tests/safety_systems/CMakeFiles/virtual_joint_stops_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : safety_systems/CMakeFiles/virtual_joint_stops_test.dir/depend

