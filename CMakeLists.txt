# ------------------------------------------------------------------------------
#                       mola_ellipse_lio_wrapper
#
# Copyright (C) 2026, Jose Luis Blanco-Claraco
# All rights reserved.
# Released under GNU GPL v3. See LICENSE file.
# See THIRD_PARTY_NOTICES.md for the licensing terms of the vendored
# EllipseLIO / i-Octree / IKFoM code under third_party/.
# ------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.22...3.31)  # Ubuntu 22.04+

if("$ENV{ROS_VERSION}" STREQUAL "2")
    set(DETECTED_ROS2 TRUE)
endif()

project(mola_ellipse_lio_wrapper LANGUAGES CXX)

# MOLA CMake scripts: "mola_xxx()"
find_package(mola_common REQUIRED)

# find dependencies:
find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED COMPONENTS common filters)
find_package(mrpt-maps REQUIRED)
find_package(mrpt-obs REQUIRED)
find_package(mrpt-tclap REQUIRED)
find_package(mola_kernel REQUIRED)
find_package(mola_yaml REQUIRED)

# ament_cmake / BUILD_TESTING must be set up before add_subdirectory() below,
# since ellipse_lio_core's own CMakeLists.txt uses ament_add_gtest():
if(DETECTED_ROS2)
  find_package(ament_cmake REQUIRED)

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    find_package(ament_cmake_gtest REQUIRED)
    ament_lint_auto_find_test_dependencies()
  endif()
endif()

# -----------------------------------------------------------------------------
#  ellipse_lio_core: the ROS-free, MOLA-free EllipseLIO algorithm library.
# -----------------------------------------------------------------------------
add_subdirectory(third_party/ellipse_lio_core)

# -----------------------------------------------------------------------------
#  mola_ellipse_lio_wrapper: the online mola-cli module
# -----------------------------------------------------------------------------
mola_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES
    module/src/EllipseLioOdometry.cpp
    module/src/EllipseLioOdometry_Adapters.cpp
    # Last:
    module/src/register.cpp
  PUBLIC_LINK_LIBRARIES
    mrpt::maps
    mrpt::obs
    mola::mola_kernel
    mola::mola_yaml
  PRIVATE_LINK_LIBRARIES
    # ellipse_lio_core is a plain (non ament/non exported) static lib: link
    # it PRIVATE so `install(EXPORT ...)` doesn't need to export it too (its
    # symbols get fully absorbed into this .so). Only
    # EllipseLioOdometry.cpp's own compilation needs its full headers (see
    # the PRIVATE include dirs below) -- the public header only needs the
    # light Types.hpp.
    ellipse_lio_core
  CMAKE_DEPENDENCIES
    mola_kernel
    mrpt-maps
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/module/include>
  # module/include/.../EllipseLioOdometry.h (installed, public) only needs
  # the light, PCL/i-Octree/IKFoM-free ellipse_lio_core/Types.hpp, so its
  # include dir is PUBLIC too: needed by any consumer of this library, in
  # particular apps/mola-ellipse-lio-cli.cpp (a separate target below).
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/ellipse_lio_core/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE
    module/src
    # Only EllipseLioOdometry.cpp needs the full
    # ellipse_lio_core::EllipseLioCore API (to construct/own an instance),
    # hence the heavier i-Octree/IKFoM include dirs stay PRIVATE rather than
    # being forced on every downstream consumer.
    third_party
    third_party/ioctree
    third_party/ikfom
)
install(
  DIRECTORY module/include/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include
)

# Install pipeline / mola-cli launch YAMLs:
install(
  DIRECTORY
    mola-cli-launchs
    pipelines
  DESTINATION
    share/${PROJECT_NAME}
)

# Install gui-* convenience scripts:
install(
  PROGRAMS
    scripts/mola-ellipselio-gui-oxford-spires
  DESTINATION
    bin
)

# -----------------------------------------------------------------------------
#  mola-ellipse-lio-cli: the offline, lossless CLI app
# -----------------------------------------------------------------------------
find_package(mola_input_mulran_dataset) # optional
find_package(mola_input_rawlog) # optional
find_package(mola_input_rosbag2) # optional

mola_add_executable(
  TARGET  mola-ellipse-lio-cli
  SOURCES apps/mola-ellipse-lio-cli.cpp
  LINK_LIBRARIES
    ${PROJECT_NAME}
    # For ellipse_lio_core/Types.hpp, transitively pulled in by the public
    # EllipseLioOdometry.h: brings PCL/Eigen/Boost include dirs and the
    # PCL_NO_PRECOMPILE define along with it (see
    # third_party/ellipse_lio_core/CMakeLists.txt).
    ellipse_lio_core
    mrpt::maps
    mrpt::tclap
)

if(mola_input_mulran_dataset_FOUND)
  message(STATUS "mola-ellipse-lio-cli: With Mulran = YES")
  target_link_libraries(mola-ellipse-lio-cli mola::mola_input_mulran_dataset)
  target_compile_definitions(mola-ellipse-lio-cli PRIVATE HAVE_MOLA_INPUT_MULRAN)
else()
  message(STATUS "mola-ellipse-lio-cli: With Mulran = NO")
endif()

if(mola_input_rawlog_FOUND)
  message(STATUS "mola-ellipse-lio-cli: With Rawlog = YES")
  target_link_libraries(mola-ellipse-lio-cli mola::mola_input_rawlog)
  target_compile_definitions(mola-ellipse-lio-cli PRIVATE HAVE_MOLA_INPUT_RAWLOG)
else()
  message(STATUS "mola-ellipse-lio-cli: With Rawlog = NO")
endif()

if(mola_input_rosbag2_FOUND)
  message(STATUS "mola-ellipse-lio-cli: With rosbag2 = YES")
  target_link_libraries(mola-ellipse-lio-cli mola::mola_input_rosbag2)
  target_compile_definitions(mola-ellipse-lio-cli PRIVATE HAVE_MOLA_INPUT_ROSBAG2)
else()
  message(STATUS "mola-ellipse-lio-cli: With rosbag2 = NO")
endif()

# -----------------------------------------------------------------------------
#  ROS2
# -----------------------------------------------------------------------------
if(DETECTED_ROS2)
  ament_package()
endif()
