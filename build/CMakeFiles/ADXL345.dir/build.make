# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.6

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
CMAKE_SOURCE_DIR = /home/pi/libsbp/c/submarino

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pi/libsbp/c/submarino/build

# Include any dependencies generated for this target.
include CMakeFiles/ADXL345.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/ADXL345.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/ADXL345.dir/flags.make

CMakeFiles/ADXL345.dir/lib/ADXL345.cpp.o: CMakeFiles/ADXL345.dir/flags.make
CMakeFiles/ADXL345.dir/lib/ADXL345.cpp.o: ../lib/ADXL345.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/libsbp/c/submarino/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/ADXL345.dir/lib/ADXL345.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ADXL345.dir/lib/ADXL345.cpp.o -c /home/pi/libsbp/c/submarino/lib/ADXL345.cpp

CMakeFiles/ADXL345.dir/lib/ADXL345.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ADXL345.dir/lib/ADXL345.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/libsbp/c/submarino/lib/ADXL345.cpp > CMakeFiles/ADXL345.dir/lib/ADXL345.cpp.i

CMakeFiles/ADXL345.dir/lib/ADXL345.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ADXL345.dir/lib/ADXL345.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/libsbp/c/submarino/lib/ADXL345.cpp -o CMakeFiles/ADXL345.dir/lib/ADXL345.cpp.s

CMakeFiles/ADXL345.dir/lib/ADXL345.cpp.o.requires:

.PHONY : CMakeFiles/ADXL345.dir/lib/ADXL345.cpp.o.requires

CMakeFiles/ADXL345.dir/lib/ADXL345.cpp.o.provides: CMakeFiles/ADXL345.dir/lib/ADXL345.cpp.o.requires
	$(MAKE) -f CMakeFiles/ADXL345.dir/build.make CMakeFiles/ADXL345.dir/lib/ADXL345.cpp.o.provides.build
.PHONY : CMakeFiles/ADXL345.dir/lib/ADXL345.cpp.o.provides

CMakeFiles/ADXL345.dir/lib/ADXL345.cpp.o.provides.build: CMakeFiles/ADXL345.dir/lib/ADXL345.cpp.o


# Object files for target ADXL345
ADXL345_OBJECTS = \
"CMakeFiles/ADXL345.dir/lib/ADXL345.cpp.o"

# External object files for target ADXL345
ADXL345_EXTERNAL_OBJECTS =

libADXL345.a: CMakeFiles/ADXL345.dir/lib/ADXL345.cpp.o
libADXL345.a: CMakeFiles/ADXL345.dir/build.make
libADXL345.a: CMakeFiles/ADXL345.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi/libsbp/c/submarino/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libADXL345.a"
	$(CMAKE_COMMAND) -P CMakeFiles/ADXL345.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ADXL345.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/ADXL345.dir/build: libADXL345.a

.PHONY : CMakeFiles/ADXL345.dir/build

CMakeFiles/ADXL345.dir/requires: CMakeFiles/ADXL345.dir/lib/ADXL345.cpp.o.requires

.PHONY : CMakeFiles/ADXL345.dir/requires

CMakeFiles/ADXL345.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ADXL345.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ADXL345.dir/clean

CMakeFiles/ADXL345.dir/depend:
	cd /home/pi/libsbp/c/submarino/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/libsbp/c/submarino /home/pi/libsbp/c/submarino /home/pi/libsbp/c/submarino/build /home/pi/libsbp/c/submarino/build /home/pi/libsbp/c/submarino/build/CMakeFiles/ADXL345.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ADXL345.dir/depend

