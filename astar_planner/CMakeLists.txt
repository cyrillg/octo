# The octo project

cmake_minimum_required (VERSION 3.15)

project(astar_planner
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

configure_file(include/${PROJECT_NAME}/${PROJECT_NAME}_config.hpp.in ${PROJECT_NAME}/${PROJECT_NAME}_config.hpp)

add_library(planner src/${PROJECT_NAME}/planner.cc src/${PROJECT_NAME}/map.cc src/${PROJECT_NAME}/cell.cc)
target_include_directories(planner
                           PUBLIC
                           "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                           "$<INSTALL_INTERFACE:include>")
target_compile_definitions(planner
                           PRIVATE "PLANNER_BUILD")
target_link_libraries(planner PUBLIC ${PROJECT_NAME}_options)


# ----------------------------------------------------------------------------
# INSTALL
# ----------------------------------------------------------------------------

install(TARGETS planner ${PROJECT_NAME}_options
        DESTINATION lib
        EXPORT plannerTargets)

install(FILES include/${PROJECT_NAME}/planner.hpp ${PROJECT_BINARY_DIR}/${PROJECT_NAME}/${PROJECT_NAME}_config.hpp
        DESTINATION include)

include(CMakePackageConfigHelpers)
# generate the config file that includes the exports
configure_package_config_file(Config.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
                              INSTALL_DESTINATION "lib/cmake/${PROJECT_NAME}"
                              NO_SET_AND_CHECK_MACRO
                              NO_CHECK_REQUIRED_COMPONENTS_MACRO)

# generate the version file for the config file
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
                                 VERSION "${${PROJECT_NAME}_VERSION_MAJOR}.${${PROJECT_NAME}_VERSION_MINOR}"
                                 COMPATIBILITY AnyNewerVersion)

# install the configuration file
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
        DESTINATION lib/cmake/${PROJECT_NAME})

install(EXPORT plannerTargets
        FILE plannerTargets.cmake
        DESTINATION lib/cmake/${PROJECT_NAME})

# allow use of the package within a project
export(EXPORT plannerTargets
       FILE ${CMAKE_CURRENT_BINARY_DIR}/plannerTargets.cmake)

# ----------------------------------------------------------------------------
# INSTALLERS
# ----------------------------------------------------------------------------

# configure installers creation with cpack
include(cmake/installers.cmake)
