# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.15

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/zx/workspace/kalibr_ws/src/calibrate_accelerometer

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/zx/workspace/kalibr_ws/src/calibrate_accelerometer/build

# Include any dependencies generated for this target.
include CMakeFiles/calibr_acc.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/calibr_acc.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/calibr_acc.dir/flags.make

CMakeFiles/calibr_acc.dir/src/main.cpp.o: CMakeFiles/calibr_acc.dir/flags.make
CMakeFiles/calibr_acc.dir/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zx/workspace/kalibr_ws/src/calibrate_accelerometer/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/calibr_acc.dir/src/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/calibr_acc.dir/src/main.cpp.o -c /home/zx/workspace/kalibr_ws/src/calibrate_accelerometer/src/main.cpp

CMakeFiles/calibr_acc.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/calibr_acc.dir/src/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zx/workspace/kalibr_ws/src/calibrate_accelerometer/src/main.cpp > CMakeFiles/calibr_acc.dir/src/main.cpp.i

CMakeFiles/calibr_acc.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/calibr_acc.dir/src/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zx/workspace/kalibr_ws/src/calibrate_accelerometer/src/main.cpp -o CMakeFiles/calibr_acc.dir/src/main.cpp.s

CMakeFiles/calibr_acc.dir/src/anker/readAnkerDataFile.cpp.o: CMakeFiles/calibr_acc.dir/flags.make
CMakeFiles/calibr_acc.dir/src/anker/readAnkerDataFile.cpp.o: ../src/anker/readAnkerDataFile.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zx/workspace/kalibr_ws/src/calibrate_accelerometer/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/calibr_acc.dir/src/anker/readAnkerDataFile.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/calibr_acc.dir/src/anker/readAnkerDataFile.cpp.o -c /home/zx/workspace/kalibr_ws/src/calibrate_accelerometer/src/anker/readAnkerDataFile.cpp

CMakeFiles/calibr_acc.dir/src/anker/readAnkerDataFile.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/calibr_acc.dir/src/anker/readAnkerDataFile.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zx/workspace/kalibr_ws/src/calibrate_accelerometer/src/anker/readAnkerDataFile.cpp > CMakeFiles/calibr_acc.dir/src/anker/readAnkerDataFile.cpp.i

CMakeFiles/calibr_acc.dir/src/anker/readAnkerDataFile.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/calibr_acc.dir/src/anker/readAnkerDataFile.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zx/workspace/kalibr_ws/src/calibrate_accelerometer/src/anker/readAnkerDataFile.cpp -o CMakeFiles/calibr_acc.dir/src/anker/readAnkerDataFile.cpp.s

CMakeFiles/calibr_acc.dir/src/calibrAcc/calibrAcc.cpp.o: CMakeFiles/calibr_acc.dir/flags.make
CMakeFiles/calibr_acc.dir/src/calibrAcc/calibrAcc.cpp.o: ../src/calibrAcc/calibrAcc.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zx/workspace/kalibr_ws/src/calibrate_accelerometer/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/calibr_acc.dir/src/calibrAcc/calibrAcc.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/calibr_acc.dir/src/calibrAcc/calibrAcc.cpp.o -c /home/zx/workspace/kalibr_ws/src/calibrate_accelerometer/src/calibrAcc/calibrAcc.cpp

CMakeFiles/calibr_acc.dir/src/calibrAcc/calibrAcc.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/calibr_acc.dir/src/calibrAcc/calibrAcc.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zx/workspace/kalibr_ws/src/calibrate_accelerometer/src/calibrAcc/calibrAcc.cpp > CMakeFiles/calibr_acc.dir/src/calibrAcc/calibrAcc.cpp.i

CMakeFiles/calibr_acc.dir/src/calibrAcc/calibrAcc.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/calibr_acc.dir/src/calibrAcc/calibrAcc.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zx/workspace/kalibr_ws/src/calibrate_accelerometer/src/calibrAcc/calibrAcc.cpp -o CMakeFiles/calibr_acc.dir/src/calibrAcc/calibrAcc.cpp.s

CMakeFiles/calibr_acc.dir/src/calibrAcc/gravityNormFactor.cpp.o: CMakeFiles/calibr_acc.dir/flags.make
CMakeFiles/calibr_acc.dir/src/calibrAcc/gravityNormFactor.cpp.o: ../src/calibrAcc/gravityNormFactor.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zx/workspace/kalibr_ws/src/calibrate_accelerometer/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/calibr_acc.dir/src/calibrAcc/gravityNormFactor.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/calibr_acc.dir/src/calibrAcc/gravityNormFactor.cpp.o -c /home/zx/workspace/kalibr_ws/src/calibrate_accelerometer/src/calibrAcc/gravityNormFactor.cpp

CMakeFiles/calibr_acc.dir/src/calibrAcc/gravityNormFactor.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/calibr_acc.dir/src/calibrAcc/gravityNormFactor.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zx/workspace/kalibr_ws/src/calibrate_accelerometer/src/calibrAcc/gravityNormFactor.cpp > CMakeFiles/calibr_acc.dir/src/calibrAcc/gravityNormFactor.cpp.i

CMakeFiles/calibr_acc.dir/src/calibrAcc/gravityNormFactor.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/calibr_acc.dir/src/calibrAcc/gravityNormFactor.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zx/workspace/kalibr_ws/src/calibrate_accelerometer/src/calibrAcc/gravityNormFactor.cpp -o CMakeFiles/calibr_acc.dir/src/calibrAcc/gravityNormFactor.cpp.s

# Object files for target calibr_acc
calibr_acc_OBJECTS = \
"CMakeFiles/calibr_acc.dir/src/main.cpp.o" \
"CMakeFiles/calibr_acc.dir/src/anker/readAnkerDataFile.cpp.o" \
"CMakeFiles/calibr_acc.dir/src/calibrAcc/calibrAcc.cpp.o" \
"CMakeFiles/calibr_acc.dir/src/calibrAcc/gravityNormFactor.cpp.o"

# External object files for target calibr_acc
calibr_acc_EXTERNAL_OBJECTS =

calibr_acc: CMakeFiles/calibr_acc.dir/src/main.cpp.o
calibr_acc: CMakeFiles/calibr_acc.dir/src/anker/readAnkerDataFile.cpp.o
calibr_acc: CMakeFiles/calibr_acc.dir/src/calibrAcc/calibrAcc.cpp.o
calibr_acc: CMakeFiles/calibr_acc.dir/src/calibrAcc/gravityNormFactor.cpp.o
calibr_acc: CMakeFiles/calibr_acc.dir/build.make
calibr_acc: /usr/local/lib/libceres.a
calibr_acc: /usr/lib/x86_64-linux-gnu/libglog.so
calibr_acc: /usr/lib/x86_64-linux-gnu/libgflags.so
calibr_acc: /usr/lib/x86_64-linux-gnu/libspqr.so
calibr_acc: /usr/lib/x86_64-linux-gnu/libtbbmalloc.so
calibr_acc: /usr/lib/x86_64-linux-gnu/libtbb.so
calibr_acc: /usr/lib/x86_64-linux-gnu/libcholmod.so
calibr_acc: /usr/lib/x86_64-linux-gnu/libccolamd.so
calibr_acc: /usr/lib/x86_64-linux-gnu/libcamd.so
calibr_acc: /usr/lib/x86_64-linux-gnu/libcolamd.so
calibr_acc: /usr/lib/x86_64-linux-gnu/libamd.so
calibr_acc: /usr/lib/liblapack.so
calibr_acc: /usr/lib/libblas.so
calibr_acc: /usr/lib/x86_64-linux-gnu/libsuitesparseconfig.so
calibr_acc: /usr/lib/x86_64-linux-gnu/librt.so
calibr_acc: /usr/lib/x86_64-linux-gnu/libcxsparse.so
calibr_acc: /usr/lib/liblapack.so
calibr_acc: /usr/lib/libblas.so
calibr_acc: /usr/lib/x86_64-linux-gnu/libsuitesparseconfig.so
calibr_acc: /usr/lib/x86_64-linux-gnu/librt.so
calibr_acc: /usr/lib/x86_64-linux-gnu/libcxsparse.so
calibr_acc: CMakeFiles/calibr_acc.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/zx/workspace/kalibr_ws/src/calibrate_accelerometer/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX executable calibr_acc"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/calibr_acc.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/calibr_acc.dir/build: calibr_acc

.PHONY : CMakeFiles/calibr_acc.dir/build

CMakeFiles/calibr_acc.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/calibr_acc.dir/cmake_clean.cmake
.PHONY : CMakeFiles/calibr_acc.dir/clean

CMakeFiles/calibr_acc.dir/depend:
	cd /home/zx/workspace/kalibr_ws/src/calibrate_accelerometer/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zx/workspace/kalibr_ws/src/calibrate_accelerometer /home/zx/workspace/kalibr_ws/src/calibrate_accelerometer /home/zx/workspace/kalibr_ws/src/calibrate_accelerometer/build /home/zx/workspace/kalibr_ws/src/calibrate_accelerometer/build /home/zx/workspace/kalibr_ws/src/calibrate_accelerometer/build/CMakeFiles/calibr_acc.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/calibr_acc.dir/depend
