# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.7

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
CMAKE_SOURCE_DIR = /home/pi/submarino/Piksi_data_logger

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pi/submarino/Piksi_data_logger/build

# Include any dependencies generated for this target.
include CMakeFiles/piksi_datalogger.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/piksi_datalogger.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/piksi_datalogger.dir/flags.make

CMakeFiles/piksi_datalogger.dir/piksi_datalogger.c.o: CMakeFiles/piksi_datalogger.dir/flags.make
CMakeFiles/piksi_datalogger.dir/piksi_datalogger.c.o: ../piksi_datalogger.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/submarino/Piksi_data_logger/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object CMakeFiles/piksi_datalogger.dir/piksi_datalogger.c.o"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/piksi_datalogger.dir/piksi_datalogger.c.o   -c /home/pi/submarino/Piksi_data_logger/piksi_datalogger.c

CMakeFiles/piksi_datalogger.dir/piksi_datalogger.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/piksi_datalogger.dir/piksi_datalogger.c.i"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/pi/submarino/Piksi_data_logger/piksi_datalogger.c > CMakeFiles/piksi_datalogger.dir/piksi_datalogger.c.i

CMakeFiles/piksi_datalogger.dir/piksi_datalogger.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/piksi_datalogger.dir/piksi_datalogger.c.s"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/pi/submarino/Piksi_data_logger/piksi_datalogger.c -o CMakeFiles/piksi_datalogger.dir/piksi_datalogger.c.s

CMakeFiles/piksi_datalogger.dir/piksi_datalogger.c.o.requires:

.PHONY : CMakeFiles/piksi_datalogger.dir/piksi_datalogger.c.o.requires

CMakeFiles/piksi_datalogger.dir/piksi_datalogger.c.o.provides: CMakeFiles/piksi_datalogger.dir/piksi_datalogger.c.o.requires
	$(MAKE) -f CMakeFiles/piksi_datalogger.dir/build.make CMakeFiles/piksi_datalogger.dir/piksi_datalogger.c.o.provides.build
.PHONY : CMakeFiles/piksi_datalogger.dir/piksi_datalogger.c.o.provides

CMakeFiles/piksi_datalogger.dir/piksi_datalogger.c.o.provides.build: CMakeFiles/piksi_datalogger.dir/piksi_datalogger.c.o


# Object files for target piksi_datalogger
piksi_datalogger_OBJECTS = \
"CMakeFiles/piksi_datalogger.dir/piksi_datalogger.c.o"

# External object files for target piksi_datalogger
piksi_datalogger_EXTERNAL_OBJECTS =

piksi_datalogger: CMakeFiles/piksi_datalogger.dir/piksi_datalogger.c.o
piksi_datalogger: CMakeFiles/piksi_datalogger.dir/build.make
piksi_datalogger: /usr/lib/arm-linux-gnueabihf/libm.so
piksi_datalogger: CMakeFiles/piksi_datalogger.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi/submarino/Piksi_data_logger/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C executable piksi_datalogger"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/piksi_datalogger.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/piksi_datalogger.dir/build: piksi_datalogger

.PHONY : CMakeFiles/piksi_datalogger.dir/build

CMakeFiles/piksi_datalogger.dir/requires: CMakeFiles/piksi_datalogger.dir/piksi_datalogger.c.o.requires

.PHONY : CMakeFiles/piksi_datalogger.dir/requires

CMakeFiles/piksi_datalogger.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/piksi_datalogger.dir/cmake_clean.cmake
.PHONY : CMakeFiles/piksi_datalogger.dir/clean

CMakeFiles/piksi_datalogger.dir/depend:
	cd /home/pi/submarino/Piksi_data_logger/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/submarino/Piksi_data_logger /home/pi/submarino/Piksi_data_logger /home/pi/submarino/Piksi_data_logger/build /home/pi/submarino/Piksi_data_logger/build /home/pi/submarino/Piksi_data_logger/build/CMakeFiles/piksi_datalogger.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/piksi_datalogger.dir/depend

