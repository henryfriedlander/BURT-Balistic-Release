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
CMAKE_SOURCE_DIR = /home/robot/proficio_toolbox

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/robot/proficio_toolbox/build

# Include any dependencies generated for this target.
include programs/CMakeFiles/homecheck.dir/depend.make

# Include the progress variables for this target.
include programs/CMakeFiles/homecheck.dir/progress.make

# Include the compile flags for this target's objects.
include programs/CMakeFiles/homecheck.dir/flags.make

programs/CMakeFiles/homecheck.dir/homecheck.cpp.o: programs/CMakeFiles/homecheck.dir/flags.make
programs/CMakeFiles/homecheck.dir/homecheck.cpp.o: ../programs/homecheck.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/robot/proficio_toolbox/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object programs/CMakeFiles/homecheck.dir/homecheck.cpp.o"
	cd /home/robot/proficio_toolbox/build/programs && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/homecheck.dir/homecheck.cpp.o -c /home/robot/proficio_toolbox/programs/homecheck.cpp

programs/CMakeFiles/homecheck.dir/homecheck.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/homecheck.dir/homecheck.cpp.i"
	cd /home/robot/proficio_toolbox/build/programs && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/robot/proficio_toolbox/programs/homecheck.cpp > CMakeFiles/homecheck.dir/homecheck.cpp.i

programs/CMakeFiles/homecheck.dir/homecheck.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/homecheck.dir/homecheck.cpp.s"
	cd /home/robot/proficio_toolbox/build/programs && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/robot/proficio_toolbox/programs/homecheck.cpp -o CMakeFiles/homecheck.dir/homecheck.cpp.s

programs/CMakeFiles/homecheck.dir/homecheck.cpp.o.requires:
.PHONY : programs/CMakeFiles/homecheck.dir/homecheck.cpp.o.requires

programs/CMakeFiles/homecheck.dir/homecheck.cpp.o.provides: programs/CMakeFiles/homecheck.dir/homecheck.cpp.o.requires
	$(MAKE) -f programs/CMakeFiles/homecheck.dir/build.make programs/CMakeFiles/homecheck.dir/homecheck.cpp.o.provides.build
.PHONY : programs/CMakeFiles/homecheck.dir/homecheck.cpp.o.provides

programs/CMakeFiles/homecheck.dir/homecheck.cpp.o.provides.build: programs/CMakeFiles/homecheck.dir/homecheck.cpp.o

# Object files for target homecheck
homecheck_OBJECTS = \
"CMakeFiles/homecheck.dir/homecheck.cpp.o"

# External object files for target homecheck
homecheck_EXTERNAL_OBJECTS =

programs/homecheck: programs/CMakeFiles/homecheck.dir/homecheck.cpp.o
programs/homecheck: /usr/lib/libCGAL_Core.so
programs/homecheck: /usr/lib/libCGAL.so
programs/homecheck: /usr/lib/x86_64-linux-gnu/libgmpxx.so
programs/homecheck: /usr/lib/x86_64-linux-gnu/libmpfr.so
programs/homecheck: /usr/lib/x86_64-linux-gnu/libgmp.so
programs/homecheck: /usr/lib/libboost_thread-mt.so
programs/homecheck: /usr/lib/libCGAL_Core.so
programs/homecheck: /usr/lib/libCGAL.so
programs/homecheck: /usr/lib/x86_64-linux-gnu/libgmpxx.so
programs/homecheck: /usr/lib/x86_64-linux-gnu/libmpfr.so
programs/homecheck: /usr/lib/x86_64-linux-gnu/libgmp.so
programs/homecheck: /usr/lib/libboost_thread-mt.so
programs/homecheck: /usr/lib/libboost_system-mt.so
programs/homecheck: /usr/lib/libboost_thread-mt.so
programs/homecheck: /usr/lib/libboost_python.so
programs/homecheck: /usr/lib/libnative.so
programs/homecheck: /usr/lib/libxenomai.so
programs/homecheck: /usr/lib/librtdm.so
programs/homecheck: /usr/lib/libpython2.7.so
programs/homecheck: src/libproficio.so.0.0.1
programs/homecheck: /usr/lib/libCGAL_Core.so
programs/homecheck: /usr/lib/libCGAL.so
programs/homecheck: /usr/lib/x86_64-linux-gnu/libgmpxx.so
programs/homecheck: /usr/lib/x86_64-linux-gnu/libmpfr.so
programs/homecheck: /usr/lib/x86_64-linux-gnu/libgmp.so
programs/homecheck: /usr/lib/libboost_thread-mt.so
programs/homecheck: programs/CMakeFiles/homecheck.dir/build.make
programs/homecheck: programs/CMakeFiles/homecheck.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable homecheck"
	cd /home/robot/proficio_toolbox/build/programs && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/homecheck.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
programs/CMakeFiles/homecheck.dir/build: programs/homecheck
.PHONY : programs/CMakeFiles/homecheck.dir/build

programs/CMakeFiles/homecheck.dir/requires: programs/CMakeFiles/homecheck.dir/homecheck.cpp.o.requires
.PHONY : programs/CMakeFiles/homecheck.dir/requires

programs/CMakeFiles/homecheck.dir/clean:
	cd /home/robot/proficio_toolbox/build/programs && $(CMAKE_COMMAND) -P CMakeFiles/homecheck.dir/cmake_clean.cmake
.PHONY : programs/CMakeFiles/homecheck.dir/clean

programs/CMakeFiles/homecheck.dir/depend:
	cd /home/robot/proficio_toolbox/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/robot/proficio_toolbox /home/robot/proficio_toolbox/programs /home/robot/proficio_toolbox/build /home/robot/proficio_toolbox/build/programs /home/robot/proficio_toolbox/build/programs/CMakeFiles/homecheck.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : programs/CMakeFiles/homecheck.dir/depend

