# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.25

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/ben/LCCV-main

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ben/LCCV-main/build

# Include any dependencies generated for this target.
include CMakeFiles/liblccv.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/liblccv.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/liblccv.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/liblccv.dir/flags.make

CMakeFiles/liblccv.dir/src/lccv.cpp.o: CMakeFiles/liblccv.dir/flags.make
CMakeFiles/liblccv.dir/src/lccv.cpp.o: /home/ben/LCCV-main/src/lccv.cpp
CMakeFiles/liblccv.dir/src/lccv.cpp.o: CMakeFiles/liblccv.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ben/LCCV-main/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/liblccv.dir/src/lccv.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/liblccv.dir/src/lccv.cpp.o -MF CMakeFiles/liblccv.dir/src/lccv.cpp.o.d -o CMakeFiles/liblccv.dir/src/lccv.cpp.o -c /home/ben/LCCV-main/src/lccv.cpp

CMakeFiles/liblccv.dir/src/lccv.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/liblccv.dir/src/lccv.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ben/LCCV-main/src/lccv.cpp > CMakeFiles/liblccv.dir/src/lccv.cpp.i

CMakeFiles/liblccv.dir/src/lccv.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/liblccv.dir/src/lccv.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ben/LCCV-main/src/lccv.cpp -o CMakeFiles/liblccv.dir/src/lccv.cpp.s

CMakeFiles/liblccv.dir/src/libcamera_app.cpp.o: CMakeFiles/liblccv.dir/flags.make
CMakeFiles/liblccv.dir/src/libcamera_app.cpp.o: /home/ben/LCCV-main/src/libcamera_app.cpp
CMakeFiles/liblccv.dir/src/libcamera_app.cpp.o: CMakeFiles/liblccv.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ben/LCCV-main/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/liblccv.dir/src/libcamera_app.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/liblccv.dir/src/libcamera_app.cpp.o -MF CMakeFiles/liblccv.dir/src/libcamera_app.cpp.o.d -o CMakeFiles/liblccv.dir/src/libcamera_app.cpp.o -c /home/ben/LCCV-main/src/libcamera_app.cpp

CMakeFiles/liblccv.dir/src/libcamera_app.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/liblccv.dir/src/libcamera_app.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ben/LCCV-main/src/libcamera_app.cpp > CMakeFiles/liblccv.dir/src/libcamera_app.cpp.i

CMakeFiles/liblccv.dir/src/libcamera_app.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/liblccv.dir/src/libcamera_app.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ben/LCCV-main/src/libcamera_app.cpp -o CMakeFiles/liblccv.dir/src/libcamera_app.cpp.s

CMakeFiles/liblccv.dir/src/libcamera_app_options.cpp.o: CMakeFiles/liblccv.dir/flags.make
CMakeFiles/liblccv.dir/src/libcamera_app_options.cpp.o: /home/ben/LCCV-main/src/libcamera_app_options.cpp
CMakeFiles/liblccv.dir/src/libcamera_app_options.cpp.o: CMakeFiles/liblccv.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ben/LCCV-main/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/liblccv.dir/src/libcamera_app_options.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/liblccv.dir/src/libcamera_app_options.cpp.o -MF CMakeFiles/liblccv.dir/src/libcamera_app_options.cpp.o.d -o CMakeFiles/liblccv.dir/src/libcamera_app_options.cpp.o -c /home/ben/LCCV-main/src/libcamera_app_options.cpp

CMakeFiles/liblccv.dir/src/libcamera_app_options.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/liblccv.dir/src/libcamera_app_options.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ben/LCCV-main/src/libcamera_app_options.cpp > CMakeFiles/liblccv.dir/src/libcamera_app_options.cpp.i

CMakeFiles/liblccv.dir/src/libcamera_app_options.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/liblccv.dir/src/libcamera_app_options.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ben/LCCV-main/src/libcamera_app_options.cpp -o CMakeFiles/liblccv.dir/src/libcamera_app_options.cpp.s

# Object files for target liblccv
liblccv_OBJECTS = \
"CMakeFiles/liblccv.dir/src/lccv.cpp.o" \
"CMakeFiles/liblccv.dir/src/libcamera_app.cpp.o" \
"CMakeFiles/liblccv.dir/src/libcamera_app_options.cpp.o"

# External object files for target liblccv
liblccv_EXTERNAL_OBJECTS =

liblccv.so: CMakeFiles/liblccv.dir/src/lccv.cpp.o
liblccv.so: CMakeFiles/liblccv.dir/src/libcamera_app.cpp.o
liblccv.so: CMakeFiles/liblccv.dir/src/libcamera_app_options.cpp.o
liblccv.so: CMakeFiles/liblccv.dir/build.make
liblccv.so: /usr/lib/aarch64-linux-gnu/libcamera.so
liblccv.so: /usr/lib/aarch64-linux-gnu/libcamera-base.so
liblccv.so: /usr/lib/aarch64-linux-gnu/libopencv_stitching.so.4.6.0
liblccv.so: /usr/lib/aarch64-linux-gnu/libopencv_alphamat.so.4.6.0
liblccv.so: /usr/lib/aarch64-linux-gnu/libopencv_aruco.so.4.6.0
liblccv.so: /usr/lib/aarch64-linux-gnu/libopencv_barcode.so.4.6.0
liblccv.so: /usr/lib/aarch64-linux-gnu/libopencv_bgsegm.so.4.6.0
liblccv.so: /usr/lib/aarch64-linux-gnu/libopencv_bioinspired.so.4.6.0
liblccv.so: /usr/lib/aarch64-linux-gnu/libopencv_ccalib.so.4.6.0
liblccv.so: /usr/lib/aarch64-linux-gnu/libopencv_cvv.so.4.6.0
liblccv.so: /usr/lib/aarch64-linux-gnu/libopencv_dnn_objdetect.so.4.6.0
liblccv.so: /usr/lib/aarch64-linux-gnu/libopencv_dnn_superres.so.4.6.0
liblccv.so: /usr/lib/aarch64-linux-gnu/libopencv_dpm.so.4.6.0
liblccv.so: /usr/lib/aarch64-linux-gnu/libopencv_face.so.4.6.0
liblccv.so: /usr/lib/aarch64-linux-gnu/libopencv_freetype.so.4.6.0
liblccv.so: /usr/lib/aarch64-linux-gnu/libopencv_fuzzy.so.4.6.0
liblccv.so: /usr/lib/aarch64-linux-gnu/libopencv_hdf.so.4.6.0
liblccv.so: /usr/lib/aarch64-linux-gnu/libopencv_hfs.so.4.6.0
liblccv.so: /usr/lib/aarch64-linux-gnu/libopencv_img_hash.so.4.6.0
liblccv.so: /usr/lib/aarch64-linux-gnu/libopencv_intensity_transform.so.4.6.0
liblccv.so: /usr/lib/aarch64-linux-gnu/libopencv_line_descriptor.so.4.6.0
liblccv.so: /usr/lib/aarch64-linux-gnu/libopencv_mcc.so.4.6.0
liblccv.so: /usr/lib/aarch64-linux-gnu/libopencv_quality.so.4.6.0
liblccv.so: /usr/lib/aarch64-linux-gnu/libopencv_rapid.so.4.6.0
liblccv.so: /usr/lib/aarch64-linux-gnu/libopencv_reg.so.4.6.0
liblccv.so: /usr/lib/aarch64-linux-gnu/libopencv_rgbd.so.4.6.0
liblccv.so: /usr/lib/aarch64-linux-gnu/libopencv_saliency.so.4.6.0
liblccv.so: /usr/lib/aarch64-linux-gnu/libopencv_shape.so.4.6.0
liblccv.so: /usr/lib/aarch64-linux-gnu/libopencv_stereo.so.4.6.0
liblccv.so: /usr/lib/aarch64-linux-gnu/libopencv_structured_light.so.4.6.0
liblccv.so: /usr/lib/aarch64-linux-gnu/libopencv_superres.so.4.6.0
liblccv.so: /usr/lib/aarch64-linux-gnu/libopencv_surface_matching.so.4.6.0
liblccv.so: /usr/lib/aarch64-linux-gnu/libopencv_tracking.so.4.6.0
liblccv.so: /usr/lib/aarch64-linux-gnu/libopencv_videostab.so.4.6.0
liblccv.so: /usr/lib/aarch64-linux-gnu/libopencv_viz.so.4.6.0
liblccv.so: /usr/lib/aarch64-linux-gnu/libopencv_wechat_qrcode.so.4.6.0
liblccv.so: /usr/lib/aarch64-linux-gnu/libopencv_xobjdetect.so.4.6.0
liblccv.so: /usr/lib/aarch64-linux-gnu/libopencv_xphoto.so.4.6.0
liblccv.so: /usr/lib/aarch64-linux-gnu/libopencv_highgui.so.4.6.0
liblccv.so: /usr/lib/aarch64-linux-gnu/libopencv_datasets.so.4.6.0
liblccv.so: /usr/lib/aarch64-linux-gnu/libopencv_plot.so.4.6.0
liblccv.so: /usr/lib/aarch64-linux-gnu/libopencv_text.so.4.6.0
liblccv.so: /usr/lib/aarch64-linux-gnu/libopencv_ml.so.4.6.0
liblccv.so: /usr/lib/aarch64-linux-gnu/libopencv_phase_unwrapping.so.4.6.0
liblccv.so: /usr/lib/aarch64-linux-gnu/libopencv_optflow.so.4.6.0
liblccv.so: /usr/lib/aarch64-linux-gnu/libopencv_ximgproc.so.4.6.0
liblccv.so: /usr/lib/aarch64-linux-gnu/libopencv_video.so.4.6.0
liblccv.so: /usr/lib/aarch64-linux-gnu/libopencv_videoio.so.4.6.0
liblccv.so: /usr/lib/aarch64-linux-gnu/libopencv_imgcodecs.so.4.6.0
liblccv.so: /usr/lib/aarch64-linux-gnu/libopencv_objdetect.so.4.6.0
liblccv.so: /usr/lib/aarch64-linux-gnu/libopencv_calib3d.so.4.6.0
liblccv.so: /usr/lib/aarch64-linux-gnu/libopencv_dnn.so.4.6.0
liblccv.so: /usr/lib/aarch64-linux-gnu/libopencv_features2d.so.4.6.0
liblccv.so: /usr/lib/aarch64-linux-gnu/libopencv_flann.so.4.6.0
liblccv.so: /usr/lib/aarch64-linux-gnu/libopencv_photo.so.4.6.0
liblccv.so: /usr/lib/aarch64-linux-gnu/libopencv_imgproc.so.4.6.0
liblccv.so: /usr/lib/aarch64-linux-gnu/libopencv_core.so.4.6.0
liblccv.so: CMakeFiles/liblccv.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ben/LCCV-main/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX shared library liblccv.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/liblccv.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/liblccv.dir/build: liblccv.so
.PHONY : CMakeFiles/liblccv.dir/build

CMakeFiles/liblccv.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/liblccv.dir/cmake_clean.cmake
.PHONY : CMakeFiles/liblccv.dir/clean

CMakeFiles/liblccv.dir/depend:
	cd /home/ben/LCCV-main/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ben/LCCV-main /home/ben/LCCV-main /home/ben/LCCV-main/build /home/ben/LCCV-main/build /home/ben/LCCV-main/build/CMakeFiles/liblccv.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/liblccv.dir/depend

