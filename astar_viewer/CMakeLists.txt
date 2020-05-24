# The octo project

cmake_minimum_required (VERSION 3.15)

project(astar_viewer
  VERSION 1.0
  DESCRIPTION ""
  LANGUAGES CXX)

# ----------------------------------------------------------------------------
# CONFIGURE
# ----------------------------------------------------------------------------

include(cmake/standard_project_settings.cmake)

# specify the C++ standard
add_library(${PROJECT_NAME}_options INTERFACE)
target_compile_features(${PROJECT_NAME}_options INTERFACE cxx_std_11)

# standard compiler warnings
add_library(${PROJECT_NAME}_warnings INTERFACE)
include(cmake/compiler_warnings.cmake)
set_project_warnings(${PROJECT_NAME}_warnings)

# enable cache system
include(cmake/cache.cmake)

# sanitizer options if supported by compiler
include(cmake/sanitizers.cmake)
enable_sanitizers(project_options)

# enable doxygen
include(cmake/doxygen.cmake)
enable_doxygen()

# allow for static analysis options
include(cmake/static_analyzers.cmake)

option(ENABLE_PCH "Enable Precompiled Headers" OFF)
if (ENABLE_PCH)
  # Global PCH parameter
  target_precompile_headers(${PROJECT_NAME}_options INTERFACE <vector> <iostream> <string> <unorderd_map> <utility>)
endif()

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")

option(BUILD_SHARED_LIBS "Build using shared libraries" OFF)

# ----------------------------------------------------------------------------
# BUILD
# ----------------------------------------------------------------------------

configure_file(include/${PROJECT_NAME}_config.hpp.in ${PROJECT_NAME}_config.hpp)

list(APPEND EXTRA_LIBS planner)

# Add source to this project's executable.
add_executable (viewer src/main.cc)
target_link_libraries(viewer PUBLIC ${EXTRA_LIBS})
target_include_directories(viewer
                           PUBLIC ${PROJECT_BINARY_DIR})

# ----------------------------------------------------------------------------
# INSTALL
# ----------------------------------------------------------------------------

install(TARGETS viewer
        DESTINATION bin)

install(FILES ${PROJECT_BINARY_DIR}/${PROJECT_NAME}_config.hpp
        DESTINATION include)

# ----------------------------------------------------------------------------
# INSTALLERS
# ----------------------------------------------------------------------------

# configure installers creation with cpack
include(cmake/installers.cmake)
