# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/yimengq/sai2/apps/cs225a/FinalProject/Airhockey

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/yimengq/sai2/apps/cs225a/FinalProject/Airhockey/build

# Include any dependencies generated for this target.
include CMakeFiles/controller.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/controller.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/controller.dir/flags.make

CMakeFiles/controller.dir/controller/controller.cpp.o: CMakeFiles/controller.dir/flags.make
CMakeFiles/controller.dir/controller/controller.cpp.o: ../controller/controller.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yimengq/sai2/apps/cs225a/FinalProject/Airhockey/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/controller.dir/controller/controller.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/controller.dir/controller/controller.cpp.o -c /home/yimengq/sai2/apps/cs225a/FinalProject/Airhockey/controller/controller.cpp

CMakeFiles/controller.dir/controller/controller.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/controller.dir/controller/controller.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yimengq/sai2/apps/cs225a/FinalProject/Airhockey/controller/controller.cpp > CMakeFiles/controller.dir/controller/controller.cpp.i

CMakeFiles/controller.dir/controller/controller.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/controller.dir/controller/controller.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yimengq/sai2/apps/cs225a/FinalProject/Airhockey/controller/controller.cpp -o CMakeFiles/controller.dir/controller/controller.cpp.s

# Object files for target controller
controller_OBJECTS = \
"CMakeFiles/controller.dir/controller/controller.cpp.o"

# External object files for target controller
controller_EXTERNAL_OBJECTS =

../bin/controller: CMakeFiles/controller.dir/controller/controller.cpp.o
../bin/controller: CMakeFiles/controller.dir/build.make
../bin/controller: /home/yimengq/sai2/core/sai2-common/build/libsai2-common.a
../bin/controller: /home/yimengq/sai2/core/chai3d/build/libchai3d.a
../bin/controller: /usr/lib/x86_64-linux-gnu/libGL.so
../bin/controller: /usr/lib/x86_64-linux-gnu/libGLU.so
../bin/controller: /home/yimengq/sai2/core/sai2-urdfreader/build/libsai2-urdf.a
../bin/controller: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
../bin/controller: /home/yimengq/sai2/core/sai2-simulation/build/libsai2-simulation.a
../bin/controller: /home/yimengq/sai2/core/sai2-model/build/libsai2-model.a
../bin/controller: /home/yimengq/sai2/core/sai2-urdfreader/build/libsai2-urdf.a
../bin/controller: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
../bin/controller: /home/yimengq/sai2/core/sai2-model/rbdl/build/librbdl.so
../bin/controller: /home/yimengq/sai2/core/sai2-graphics/build/libsai2-graphics.a
../bin/controller: /home/yimengq/sai2/core/sai2-urdfreader/build/libsai2-urdf.a
../bin/controller: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
../bin/controller: /home/yimengq/sai2/core/chai3d/build/libchai3d.a
../bin/controller: /usr/lib/x86_64-linux-gnu/libGL.so
../bin/controller: /usr/lib/x86_64-linux-gnu/libGLU.so
../bin/controller: /usr/lib/x86_64-linux-gnu/libhiredis.so
../bin/controller: /usr/lib/x86_64-linux-gnu/libglfw.so
../bin/controller: /home/yimengq/sai2/core/sai2-primitives/build/libsai2-primitives.a
../bin/controller: /home/yimengq/sai2/core/sai2-primitives/../external/ReflexxesTypeII/Linux/x64/release/lib/shared/libReflexxesTypeII.so
../bin/controller: /home/yimengq/sai2/core/sai2-common/build/libsai2-common.a
../bin/controller: /home/yimengq/sai2/core/chai3d/build/libchai3d.a
../bin/controller: /usr/lib/x86_64-linux-gnu/libGL.so
../bin/controller: /usr/lib/x86_64-linux-gnu/libGLU.so
../bin/controller: /home/yimengq/sai2/core/sai2-urdfreader/build/libsai2-urdf.a
../bin/controller: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
../bin/controller: /home/yimengq/sai2/core/sai2-simulation/build/libsai2-simulation.a
../bin/controller: /home/yimengq/sai2/core/sai2-model/build/libsai2-model.a
../bin/controller: /home/yimengq/sai2/core/sai2-urdfreader/build/libsai2-urdf.a
../bin/controller: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
../bin/controller: /home/yimengq/sai2/core/sai2-model/rbdl/build/librbdl.so
../bin/controller: /home/yimengq/sai2/core/sai2-graphics/build/libsai2-graphics.a
../bin/controller: /home/yimengq/sai2/core/sai2-urdfreader/build/libsai2-urdf.a
../bin/controller: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
../bin/controller: /home/yimengq/sai2/core/chai3d/build/libchai3d.a
../bin/controller: /usr/lib/x86_64-linux-gnu/libGL.so
../bin/controller: /usr/lib/x86_64-linux-gnu/libGLU.so
../bin/controller: /usr/lib/x86_64-linux-gnu/libhiredis.so
../bin/controller: /usr/lib/x86_64-linux-gnu/libglfw.so
../bin/controller: /home/yimengq/sai2/core/sai2-primitives/../external/ReflexxesTypeII/Linux/x64/release/lib/shared/libReflexxesTypeII.so
../bin/controller: CMakeFiles/controller.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/yimengq/sai2/apps/cs225a/FinalProject/Airhockey/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../bin/controller"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/controller.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/controller.dir/build: ../bin/controller

.PHONY : CMakeFiles/controller.dir/build

CMakeFiles/controller.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/controller.dir/cmake_clean.cmake
.PHONY : CMakeFiles/controller.dir/clean

CMakeFiles/controller.dir/depend:
	cd /home/yimengq/sai2/apps/cs225a/FinalProject/Airhockey/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/yimengq/sai2/apps/cs225a/FinalProject/Airhockey /home/yimengq/sai2/apps/cs225a/FinalProject/Airhockey /home/yimengq/sai2/apps/cs225a/FinalProject/Airhockey/build /home/yimengq/sai2/apps/cs225a/FinalProject/Airhockey/build /home/yimengq/sai2/apps/cs225a/FinalProject/Airhockey/build/CMakeFiles/controller.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/controller.dir/depend

