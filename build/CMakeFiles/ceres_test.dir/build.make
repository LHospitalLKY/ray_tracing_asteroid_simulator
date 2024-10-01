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
include CMakeFiles/ceres_test.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/ceres_test.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/ceres_test.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/ceres_test.dir/flags.make

CMakeFiles/ceres_test.dir/test/ceres_test.cpp.o: CMakeFiles/ceres_test.dir/flags.make
CMakeFiles/ceres_test.dir/test/ceres_test.cpp.o: /home/lho/MyProgramm/2024.04/g2o_test/test/ceres_test.cpp
CMakeFiles/ceres_test.dir/test/ceres_test.cpp.o: CMakeFiles/ceres_test.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/lho/MyProgramm/2024.04/g2o_test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/ceres_test.dir/test/ceres_test.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/ceres_test.dir/test/ceres_test.cpp.o -MF CMakeFiles/ceres_test.dir/test/ceres_test.cpp.o.d -o CMakeFiles/ceres_test.dir/test/ceres_test.cpp.o -c /home/lho/MyProgramm/2024.04/g2o_test/test/ceres_test.cpp

CMakeFiles/ceres_test.dir/test/ceres_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/ceres_test.dir/test/ceres_test.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lho/MyProgramm/2024.04/g2o_test/test/ceres_test.cpp > CMakeFiles/ceres_test.dir/test/ceres_test.cpp.i

CMakeFiles/ceres_test.dir/test/ceres_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/ceres_test.dir/test/ceres_test.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lho/MyProgramm/2024.04/g2o_test/test/ceres_test.cpp -o CMakeFiles/ceres_test.dir/test/ceres_test.cpp.s

# Object files for target ceres_test
ceres_test_OBJECTS = \
"CMakeFiles/ceres_test.dir/test/ceres_test.cpp.o"

# External object files for target ceres_test
ceres_test_EXTERNAL_OBJECTS =

ceres_test: CMakeFiles/ceres_test.dir/test/ceres_test.cpp.o
ceres_test: CMakeFiles/ceres_test.dir/build.make
ceres_test: libbright_calcu.a
ceres_test: /usr/local/lib/libceres.a
ceres_test: /usr/lib/x86_64-linux-gnu/libglog.so.0.4.0
ceres_test: /usr/lib/x86_64-linux-gnu/libunwind.so
ceres_test: /usr/lib/x86_64-linux-gnu/libgflags.so.2.2.2
ceres_test: /usr/lib/x86_64-linux-gnu/libspqr.so
ceres_test: /usr/lib/x86_64-linux-gnu/libtbb.so.2
ceres_test: /usr/lib/x86_64-linux-gnu/libcholmod.so
ceres_test: /usr/lib/x86_64-linux-gnu/libamd.so
ceres_test: /usr/lib/x86_64-linux-gnu/libcamd.so
ceres_test: /usr/lib/x86_64-linux-gnu/libccolamd.so
ceres_test: /usr/lib/x86_64-linux-gnu/libcolamd.so
ceres_test: /usr/lib/x86_64-linux-gnu/libsuitesparseconfig.so
ceres_test: /usr/lib/x86_64-linux-gnu/libmetis.so
ceres_test: /usr/local/cuda-12.4/targets/x86_64-linux/lib/libcudart.so
ceres_test: /usr/lib/x86_64-linux-gnu/librt.a
ceres_test: /usr/local/cuda-12.4/targets/x86_64-linux/lib/libcusolver.so
ceres_test: /usr/local/cuda-12.4/targets/x86_64-linux/lib/libcublas.so
ceres_test: /usr/local/cuda-12.4/targets/x86_64-linux/lib/libculibos.a
ceres_test: /usr/local/cuda-12.4/targets/x86_64-linux/lib/libcublasLt.so
ceres_test: /usr/local/cuda-12.4/targets/x86_64-linux/lib/libcusparse.so
ceres_test: /usr/local/cuda-12.4/targets/x86_64-linux/lib/libnvJitLink.so
ceres_test: /usr/local/lib/libceres_cuda_kernels.a
ceres_test: /usr/lib/x86_64-linux-gnu/libmkl_intel_lp64.so
ceres_test: /usr/lib/x86_64-linux-gnu/libmkl_intel_thread.so
ceres_test: /usr/lib/x86_64-linux-gnu/libmkl_core.so
ceres_test: /usr/lib/x86_64-linux-gnu/libiomp5.so
ceres_test: CMakeFiles/ceres_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/lho/MyProgramm/2024.04/g2o_test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ceres_test"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ceres_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/ceres_test.dir/build: ceres_test
.PHONY : CMakeFiles/ceres_test.dir/build

CMakeFiles/ceres_test.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ceres_test.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ceres_test.dir/clean

CMakeFiles/ceres_test.dir/depend:
	cd /home/lho/MyProgramm/2024.04/g2o_test/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lho/MyProgramm/2024.04/g2o_test /home/lho/MyProgramm/2024.04/g2o_test /home/lho/MyProgramm/2024.04/g2o_test/build /home/lho/MyProgramm/2024.04/g2o_test/build /home/lho/MyProgramm/2024.04/g2o_test/build/CMakeFiles/ceres_test.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/ceres_test.dir/depend
