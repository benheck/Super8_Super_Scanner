cmake_minimum_required(VERSION 3.6)

# Project name
project(scanner)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set CMake prefix path
set(CMAKE_PREFIX_PATH ${CMAKE_SOURCE_DIR}/../build)

# Find required packages
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Qt5Widgets REQUIRED)
pkg_check_modules(LIBCAMERA REQUIRED libcamera)
find_package(Boost REQUIRED COMPONENTS system)

# Include directories for LCCV, Qt5, and dependencies
include_directories(
    ${LIBCAMERA_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/../include # Add the local lccv include directory
)

# Link directories
link_directories(${CMAKE_SOURCE_DIR}/../build)

# Define the executable
add_executable(scanner
    src/main.cpp
    src/MarlinController.cpp # Add MarlinController.cpp to the build
)

# Link the executable with required libraries
target_link_libraries(scanner
    PRIVATE
    Qt5::Widgets
    ${CMAKE_SOURCE_DIR}/../build/liblccv.so # Explicitly link the local lccv library
    ${OpenCV_LIBS}
    ${LIBCAMERA_LINK_LIBRARIES}
    Boost::system
)

# Ensure proper compiler definitions for Qt
target_compile_definitions(scanner PRIVATE ${Qt5Widgets_DEFINITIONS})

# Ensure proper compiler flags for Qt
set_target_properties(scanner PROPERTIES
    AUTOMOC ON
    AUTOUIC ON
    AUTORCC ON
)

# Add Boost system library explicitly if needed
if(Boost_FOUND)
    target_include_directories(scanner PRIVATE ${Boost_INCLUDE_DIRS})
    target_link_libraries(scanner PRIVATE Boost::system)
endif()

# Add libcamera library explicitly if needed
if(LIBCAMERA_FOUND)
    target_include_directories(scanner PRIVATE ${LIBCAMERA_INCLUDE_DIRS})
    target_link_libraries(scanner PRIVATE ${LIBCAMERA_LIBRARIES})
endif()