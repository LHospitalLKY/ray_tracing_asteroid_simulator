# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.29

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
CMAKE_COMMAND = /usr/local/cmake/bin/cmake

# The command to remove a file.
RM = /usr/local/cmake/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/lho/MyProgramm/2024.04/g2o_test

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/lho/MyProgramm/2024.04/g2o_test/build

# Include any dependencies generated for this target.
include CMakeFiles/damit_obj_converter_test.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/damit_obj_converter_test.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/damit_obj_converter_test.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/damit_obj_converter_test.dir/flags.make

CMakeFiles/damit_obj_converter_test.dir/src/common.cpp.o: CMakeFiles/damit_obj_converter_test.dir/flags.make
CMakeFiles/damit_obj_converter_test.dir/src/common.cpp.o: /home/lho/MyProgramm/2024.04/g2o_test/src/common.cpp
CMakeFiles/damit_obj_converter_test.dir/src/common.cpp.o: CMakeFiles/damit_obj_converter_test.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/lho/MyProgramm/2024.04/g2o_test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/damit_obj_converter_test.dir/src/common.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/damit_obj_converter_test.dir/src/common.cpp.o -MF CMakeFiles/damit_obj_converter_test.dir/src/common.cpp.o.d -o CMakeFiles/damit_obj_converter_test.dir/src/common.cpp.o -c /home/lho/MyProgramm/2024.04/g2o_test/src/common.cpp

CMakeFiles/damit_obj_converter_test.dir/src/common.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/damit_obj_converter_test.dir/src/common.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lho/MyProgramm/2024.04/g2o_test/src/common.cpp > CMakeFiles/damit_obj_converter_test.dir/src/common.cpp.i

CMakeFiles/damit_obj_converter_test.dir/src/common.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/damit_obj_converter_test.dir/src/common.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lho/MyProgramm/2024.04/g2o_test/src/common.cpp -o CMakeFiles/damit_obj_converter_test.dir/src/common.cpp.s

CMakeFiles/damit_obj_converter_test.dir/src/damit_obj_converter.cpp.o: CMakeFiles/damit_obj_converter_test.dir/flags.make
CMakeFiles/damit_obj_converter_test.dir/src/damit_obj_converter.cpp.o: /home/lho/MyProgramm/2024.04/g2o_test/src/damit_obj_converter.cpp
CMakeFiles/damit_obj_converter_test.dir/src/damit_obj_converter.cpp.o: CMakeFiles/damit_obj_converter_test.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/lho/MyProgramm/2024.04/g2o_test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/damit_obj_converter_test.dir/src/damit_obj_converter.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/damit_obj_converter_test.dir/src/damit_obj_converter.cpp.o -MF CMakeFiles/damit_obj_converter_test.dir/src/damit_obj_converter.cpp.o.d -o CMakeFiles/damit_obj_converter_test.dir/src/damit_obj_converter.cpp.o -c /home/lho/MyProgramm/2024.04/g2o_test/src/damit_obj_converter.cpp

CMakeFiles/damit_obj_converter_test.dir/src/damit_obj_converter.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/damit_obj_converter_test.dir/src/damit_obj_converter.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lho/MyProgramm/2024.04/g2o_test/src/damit_obj_converter.cpp > CMakeFiles/damit_obj_converter_test.dir/src/damit_obj_converter.cpp.i

CMakeFiles/damit_obj_converter_test.dir/src/damit_obj_converter.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/damit_obj_converter_test.dir/src/damit_obj_converter.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lho/MyProgramm/2024.04/g2o_test/src/damit_obj_converter.cpp -o CMakeFiles/damit_obj_converter_test.dir/src/damit_obj_converter.cpp.s

CMakeFiles/damit_obj_converter_test.dir/test/damit_obj_converter_test.cpp.o: CMakeFiles/damit_obj_converter_test.dir/flags.make
CMakeFiles/damit_obj_converter_test.dir/test/damit_obj_converter_test.cpp.o: /home/lho/MyProgramm/2024.04/g2o_test/test/damit_obj_converter_test.cpp
CMakeFiles/damit_obj_converter_test.dir/test/damit_obj_converter_test.cpp.o: CMakeFiles/damit_obj_converter_test.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/lho/MyProgramm/2024.04/g2o_test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/damit_obj_converter_test.dir/test/damit_obj_converter_test.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/damit_obj_converter_test.dir/test/damit_obj_converter_test.cpp.o -MF CMakeFiles/damit_obj_converter_test.dir/test/damit_obj_converter_test.cpp.o.d -o CMakeFiles/damit_obj_converter_test.dir/test/damit_obj_converter_test.cpp.o -c /home/lho/MyProgramm/2024.04/g2o_test/test/damit_obj_converter_test.cpp

CMakeFiles/damit_obj_converter_test.dir/test/damit_obj_converter_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/damit_obj_converter_test.dir/test/damit_obj_converter_test.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lho/MyProgramm/2024.04/g2o_test/test/damit_obj_converter_test.cpp > CMakeFiles/damit_obj_converter_test.dir/test/damit_obj_converter_test.cpp.i

CMakeFiles/damit_obj_converter_test.dir/test/damit_obj_converter_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/damit_obj_converter_test.dir/test/damit_obj_converter_test.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lho/MyProgramm/2024.04/g2o_test/test/damit_obj_converter_test.cpp -o CMakeFiles/damit_obj_converter_test.dir/test/damit_obj_converter_test.cpp.s

# Object files for target damit_obj_converter_test
damit_obj_converter_test_OBJECTS = \
"CMakeFiles/damit_obj_converter_test.dir/src/common.cpp.o" \
"CMakeFiles/damit_obj_converter_test.dir/src/damit_obj_converter.cpp.o" \
"CMakeFiles/damit_obj_converter_test.dir/test/damit_obj_converter_test.cpp.o"

# External object files for target damit_obj_converter_test
damit_obj_converter_test_EXTERNAL_OBJECTS =

damit_obj_converter_test: CMakeFiles/damit_obj_converter_test.dir/src/common.cpp.o
damit_obj_converter_test: CMakeFiles/damit_obj_converter_test.dir/src/damit_obj_converter.cpp.o
damit_obj_converter_test: CMakeFiles/damit_obj_converter_test.dir/test/damit_obj_converter_test.cpp.o
damit_obj_converter_test: CMakeFiles/damit_obj_converter_test.dir/build.make
damit_obj_converter_test: CMakeFiles/damit_obj_converter_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/lho/MyProgramm/2024.04/g2o_test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable damit_obj_converter_test"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/damit_obj_converter_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/damit_obj_converter_test.dir/build: damit_obj_converter_test
.PHONY : CMakeFiles/damit_obj_converter_test.dir/build

CMakeFiles/damit_obj_converter_test.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/damit_obj_converter_test.dir/cmake_clean.cmake
.PHONY : CMakeFiles/damit_obj_converter_test.dir/clean

CMakeFiles/damit_obj_converter_test.dir/depend:
	cd /home/lho/MyProgramm/2024.04/g2o_test/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lho/MyProgramm/2024.04/g2o_test /home/lho/MyProgramm/2024.04/g2o_test /home/lho/MyProgramm/2024.04/g2o_test/build /home/lho/MyProgramm/2024.04/g2o_test/build /home/lho/MyProgramm/2024.04/g2o_test/build/CMakeFiles/damit_obj_converter_test.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/damit_obj_converter_test.dir/depend

