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
CMAKE_SOURCE_DIR = "E:\NTI\Brightskies_Internship\Task 4\Client_test"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "E:\NTI\Brightskies_Internship\Task 4\Client_test\build"

# Include any dependencies generated for this target.
include CMakeFiles/Client_Test_App.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/Client_Test_App.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/Client_Test_App.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Client_Test_App.dir/flags.make

CMakeFiles/Client_Test_App.dir/Client_Test_App.cpp.obj: CMakeFiles/Client_Test_App.dir/flags.make
CMakeFiles/Client_Test_App.dir/Client_Test_App.cpp.obj: CMakeFiles/Client_Test_App.dir/includes_CXX.rsp
CMakeFiles/Client_Test_App.dir/Client_Test_App.cpp.obj: E:/NTI/Brightskies_Internship/Task\ 4/Client_test/Client_Test_App.cpp
CMakeFiles/Client_Test_App.dir/Client_Test_App.cpp.obj: CMakeFiles/Client_Test_App.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir="E:\NTI\Brightskies_Internship\Task 4\Client_test\build\CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Client_Test_App.dir/Client_Test_App.cpp.obj"
	"D:\Courses\ES\MinGW 64\mingw64\bin\c++.exe" $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/Client_Test_App.dir/Client_Test_App.cpp.obj -MF CMakeFiles\Client_Test_App.dir\Client_Test_App.cpp.obj.d -o CMakeFiles\Client_Test_App.dir\Client_Test_App.cpp.obj -c "E:\NTI\Brightskies_Internship\Task 4\Client_test\Client_Test_App.cpp"

CMakeFiles/Client_Test_App.dir/Client_Test_App.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/Client_Test_App.dir/Client_Test_App.cpp.i"
	"D:\Courses\ES\MinGW 64\mingw64\bin\c++.exe" $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "E:\NTI\Brightskies_Internship\Task 4\Client_test\Client_Test_App.cpp" > CMakeFiles\Client_Test_App.dir\Client_Test_App.cpp.i

CMakeFiles/Client_Test_App.dir/Client_Test_App.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/Client_Test_App.dir/Client_Test_App.cpp.s"
	"D:\Courses\ES\MinGW 64\mingw64\bin\c++.exe" $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "E:\NTI\Brightskies_Internship\Task 4\Client_test\Client_Test_App.cpp" -o CMakeFiles\Client_Test_App.dir\Client_Test_App.cpp.s

# Object files for target Client_Test_App
Client_Test_App_OBJECTS = \
"CMakeFiles/Client_Test_App.dir/Client_Test_App.cpp.obj"

# External object files for target Client_Test_App
Client_Test_App_EXTERNAL_OBJECTS =

Client_Test_App.exe: CMakeFiles/Client_Test_App.dir/Client_Test_App.cpp.obj
Client_Test_App.exe: CMakeFiles/Client_Test_App.dir/build.make
Client_Test_App.exe: lib/libgtest.a
Client_Test_App.exe: lib/libgtest_main.a
Client_Test_App.exe: libAPP_UDS_Diag.a
Client_Test_App.exe: lib/libgtest.a
Client_Test_App.exe: CMakeFiles/Client_Test_App.dir/linkLibs.rsp
Client_Test_App.exe: CMakeFiles/Client_Test_App.dir/objects1.rsp
Client_Test_App.exe: CMakeFiles/Client_Test_App.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir="E:\NTI\Brightskies_Internship\Task 4\Client_test\build\CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable Client_Test_App.exe"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles\Client_Test_App.dir\link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Client_Test_App.dir/build: Client_Test_App.exe
.PHONY : CMakeFiles/Client_Test_App.dir/build

CMakeFiles/Client_Test_App.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles\Client_Test_App.dir\cmake_clean.cmake
.PHONY : CMakeFiles/Client_Test_App.dir/clean

CMakeFiles/Client_Test_App.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" "E:\NTI\Brightskies_Internship\Task 4\Client_test" "E:\NTI\Brightskies_Internship\Task 4\Client_test" "E:\NTI\Brightskies_Internship\Task 4\Client_test\build" "E:\NTI\Brightskies_Internship\Task 4\Client_test\build" "E:\NTI\Brightskies_Internship\Task 4\Client_test\build\CMakeFiles\Client_Test_App.dir\DependInfo.cmake" "--color=$(COLOR)"
.PHONY : CMakeFiles/Client_Test_App.dir/depend

