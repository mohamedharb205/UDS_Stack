# CMAKE generated file: DO NOT EDIT!
# Generated by "MinGW Makefiles" Generator, CMake Version 3.27

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

SHELL = cmd.exe

# The CMake executable.
CMAKE_COMMAND = "D:\Courses\ES\MinGW 64\mingw64\bin\cmake.exe"

# The command to remove a file.
RM = "D:\Courses\ES\MinGW 64\mingw64\bin\cmake.exe" -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = "E:\NTI\Brightskies_Internship\Task 4\Client_Request_Test"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "E:\NTI\Brightskies_Internship\Task 4\Client_Request_Test\build"

# Include any dependencies generated for this target.
include CMakeFiles/APP_UDS_Diag.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/APP_UDS_Diag.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/APP_UDS_Diag.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/APP_UDS_Diag.dir/flags.make

CMakeFiles/APP_UDS_Diag.dir/APP_UDS_Diag.c.obj: CMakeFiles/APP_UDS_Diag.dir/flags.make
CMakeFiles/APP_UDS_Diag.dir/APP_UDS_Diag.c.obj: E:/NTI/Brightskies_Internship/Task\ 4/Client_Request_Test/APP_UDS_Diag.c
CMakeFiles/APP_UDS_Diag.dir/APP_UDS_Diag.c.obj: CMakeFiles/APP_UDS_Diag.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir="E:\NTI\Brightskies_Internship\Task 4\Client_Request_Test\build\CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building C object CMakeFiles/APP_UDS_Diag.dir/APP_UDS_Diag.c.obj"
	"D:\Courses\ES\MinGW 64\mingw64\bin\gcc.exe" $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/APP_UDS_Diag.dir/APP_UDS_Diag.c.obj -MF CMakeFiles\APP_UDS_Diag.dir\APP_UDS_Diag.c.obj.d -o CMakeFiles\APP_UDS_Diag.dir\APP_UDS_Diag.c.obj -c "E:\NTI\Brightskies_Internship\Task 4\Client_Request_Test\APP_UDS_Diag.c"

CMakeFiles/APP_UDS_Diag.dir/APP_UDS_Diag.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing C source to CMakeFiles/APP_UDS_Diag.dir/APP_UDS_Diag.c.i"
	"D:\Courses\ES\MinGW 64\mingw64\bin\gcc.exe" $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E "E:\NTI\Brightskies_Internship\Task 4\Client_Request_Test\APP_UDS_Diag.c" > CMakeFiles\APP_UDS_Diag.dir\APP_UDS_Diag.c.i

CMakeFiles/APP_UDS_Diag.dir/APP_UDS_Diag.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling C source to assembly CMakeFiles/APP_UDS_Diag.dir/APP_UDS_Diag.c.s"
	"D:\Courses\ES\MinGW 64\mingw64\bin\gcc.exe" $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S "E:\NTI\Brightskies_Internship\Task 4\Client_Request_Test\APP_UDS_Diag.c" -o CMakeFiles\APP_UDS_Diag.dir\APP_UDS_Diag.c.s

# Object files for target APP_UDS_Diag
APP_UDS_Diag_OBJECTS = \
"CMakeFiles/APP_UDS_Diag.dir/APP_UDS_Diag.c.obj"

# External object files for target APP_UDS_Diag
APP_UDS_Diag_EXTERNAL_OBJECTS =

libAPP_UDS_Diag.a: CMakeFiles/APP_UDS_Diag.dir/APP_UDS_Diag.c.obj
libAPP_UDS_Diag.a: CMakeFiles/APP_UDS_Diag.dir/build.make
libAPP_UDS_Diag.a: CMakeFiles/APP_UDS_Diag.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir="E:\NTI\Brightskies_Internship\Task 4\Client_Request_Test\build\CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Linking C static library libAPP_UDS_Diag.a"
	$(CMAKE_COMMAND) -P CMakeFiles\APP_UDS_Diag.dir\cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles\APP_UDS_Diag.dir\link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/APP_UDS_Diag.dir/build: libAPP_UDS_Diag.a
.PHONY : CMakeFiles/APP_UDS_Diag.dir/build

CMakeFiles/APP_UDS_Diag.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles\APP_UDS_Diag.dir\cmake_clean.cmake
.PHONY : CMakeFiles/APP_UDS_Diag.dir/clean

CMakeFiles/APP_UDS_Diag.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" "E:\NTI\Brightskies_Internship\Task 4\Client_Request_Test" "E:\NTI\Brightskies_Internship\Task 4\Client_Request_Test" "E:\NTI\Brightskies_Internship\Task 4\Client_Request_Test\build" "E:\NTI\Brightskies_Internship\Task 4\Client_Request_Test\build" "E:\NTI\Brightskies_Internship\Task 4\Client_Request_Test\build\CMakeFiles\APP_UDS_Diag.dir\DependInfo.cmake" "--color=$(COLOR)"
.PHONY : CMakeFiles/APP_UDS_Diag.dir/depend

