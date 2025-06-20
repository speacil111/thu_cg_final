# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Default target executed when no arguments are given to make.
default_target: all
.PHONY : default_target

# Allow only one "make -f Makefile2" at a time, but pass parallelism.
.NOTPARALLEL:

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /mnt/c/Users/speacil/Desktop/计图大作业/thu_cg_final

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /mnt/c/Users/speacil/Desktop/计图大作业/thu_cg_final

#=============================================================================
# Targets provided globally by CMake.

# Special rule for the target edit_cache
edit_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "No interactive CMake dialog available..."
	/usr/bin/cmake -E echo No\ interactive\ CMake\ dialog\ available.
.PHONY : edit_cache

# Special rule for the target edit_cache
edit_cache/fast: edit_cache
.PHONY : edit_cache/fast

# Special rule for the target rebuild_cache
rebuild_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake to regenerate build system..."
	/usr/bin/cmake --regenerate-during-build -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : rebuild_cache

# Special rule for the target rebuild_cache
rebuild_cache/fast: rebuild_cache
.PHONY : rebuild_cache/fast

# The main all target
all: cmake_check_build_system
	$(CMAKE_COMMAND) -E cmake_progress_start /mnt/c/Users/speacil/Desktop/计图大作业/thu_cg_final/CMakeFiles /mnt/c/Users/speacil/Desktop/计图大作业/thu_cg_final//CMakeFiles/progress.marks
	$(MAKE) $(MAKESILENT) -f CMakeFiles/Makefile2 all
	$(CMAKE_COMMAND) -E cmake_progress_start /mnt/c/Users/speacil/Desktop/计图大作业/thu_cg_final/CMakeFiles 0
.PHONY : all

# The main clean target
clean:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/Makefile2 clean
.PHONY : clean

# The main clean target
clean/fast: clean
.PHONY : clean/fast

# Prepare targets for installation.
preinstall: all
	$(MAKE) $(MAKESILENT) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall

# Prepare targets for installation.
preinstall/fast:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall/fast

# clear depends
depend:
	$(CMAKE_COMMAND) -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 1
.PHONY : depend

#=============================================================================
# Target rules for targets named PA1

# Build rule for target.
PA1: cmake_check_build_system
	$(MAKE) $(MAKESILENT) -f CMakeFiles/Makefile2 PA1
.PHONY : PA1

# fast build rule for target.
PA1/fast:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/PA1.dir/build.make CMakeFiles/PA1.dir/build
.PHONY : PA1/fast

#=============================================================================
# Target rules for targets named vecmath

# Build rule for target.
vecmath: cmake_check_build_system
	$(MAKE) $(MAKESILENT) -f CMakeFiles/Makefile2 vecmath
.PHONY : vecmath

# fast build rule for target.
vecmath/fast:
	$(MAKE) $(MAKESILENT) -f deps/vecmath/CMakeFiles/vecmath.dir/build.make deps/vecmath/CMakeFiles/vecmath.dir/build
.PHONY : vecmath/fast

src/image.o: src/image.cpp.o
.PHONY : src/image.o

# target to build an object file
src/image.cpp.o:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/PA1.dir/build.make CMakeFiles/PA1.dir/src/image.cpp.o
.PHONY : src/image.cpp.o

src/image.i: src/image.cpp.i
.PHONY : src/image.i

# target to preprocess a source file
src/image.cpp.i:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/PA1.dir/build.make CMakeFiles/PA1.dir/src/image.cpp.i
.PHONY : src/image.cpp.i

src/image.s: src/image.cpp.s
.PHONY : src/image.s

# target to generate assembly for a file
src/image.cpp.s:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/PA1.dir/build.make CMakeFiles/PA1.dir/src/image.cpp.s
.PHONY : src/image.cpp.s

src/main.o: src/main.cpp.o
.PHONY : src/main.o

# target to build an object file
src/main.cpp.o:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/PA1.dir/build.make CMakeFiles/PA1.dir/src/main.cpp.o
.PHONY : src/main.cpp.o

src/main.i: src/main.cpp.i
.PHONY : src/main.i

# target to preprocess a source file
src/main.cpp.i:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/PA1.dir/build.make CMakeFiles/PA1.dir/src/main.cpp.i
.PHONY : src/main.cpp.i

src/main.s: src/main.cpp.s
.PHONY : src/main.s

# target to generate assembly for a file
src/main.cpp.s:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/PA1.dir/build.make CMakeFiles/PA1.dir/src/main.cpp.s
.PHONY : src/main.cpp.s

src/mesh.o: src/mesh.cpp.o
.PHONY : src/mesh.o

# target to build an object file
src/mesh.cpp.o:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/PA1.dir/build.make CMakeFiles/PA1.dir/src/mesh.cpp.o
.PHONY : src/mesh.cpp.o

src/mesh.i: src/mesh.cpp.i
.PHONY : src/mesh.i

# target to preprocess a source file
src/mesh.cpp.i:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/PA1.dir/build.make CMakeFiles/PA1.dir/src/mesh.cpp.i
.PHONY : src/mesh.cpp.i

src/mesh.s: src/mesh.cpp.s
.PHONY : src/mesh.s

# target to generate assembly for a file
src/mesh.cpp.s:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/PA1.dir/build.make CMakeFiles/PA1.dir/src/mesh.cpp.s
.PHONY : src/mesh.cpp.s

src/scene_parser.o: src/scene_parser.cpp.o
.PHONY : src/scene_parser.o

# target to build an object file
src/scene_parser.cpp.o:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/PA1.dir/build.make CMakeFiles/PA1.dir/src/scene_parser.cpp.o
.PHONY : src/scene_parser.cpp.o

src/scene_parser.i: src/scene_parser.cpp.i
.PHONY : src/scene_parser.i

# target to preprocess a source file
src/scene_parser.cpp.i:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/PA1.dir/build.make CMakeFiles/PA1.dir/src/scene_parser.cpp.i
.PHONY : src/scene_parser.cpp.i

src/scene_parser.s: src/scene_parser.cpp.s
.PHONY : src/scene_parser.s

# target to generate assembly for a file
src/scene_parser.cpp.s:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/PA1.dir/build.make CMakeFiles/PA1.dir/src/scene_parser.cpp.s
.PHONY : src/scene_parser.cpp.s

# Help Target
help:
	@echo "The following are some of the valid targets for this Makefile:"
	@echo "... all (the default if no target is provided)"
	@echo "... clean"
	@echo "... depend"
	@echo "... edit_cache"
	@echo "... rebuild_cache"
	@echo "... PA1"
	@echo "... vecmath"
	@echo "... src/image.o"
	@echo "... src/image.i"
	@echo "... src/image.s"
	@echo "... src/main.o"
	@echo "... src/main.i"
	@echo "... src/main.s"
	@echo "... src/mesh.o"
	@echo "... src/mesh.i"
	@echo "... src/mesh.s"
	@echo "... src/scene_parser.o"
	@echo "... src/scene_parser.i"
	@echo "... src/scene_parser.s"
.PHONY : help



#=============================================================================
# Special targets to cleanup operation of make.

# Special rule to run CMake to check the build system integrity.
# No rule that depends on this can have commands that come from listfiles
# because they might be regenerated.
cmake_check_build_system:
	$(CMAKE_COMMAND) -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 0
.PHONY : cmake_check_build_system

