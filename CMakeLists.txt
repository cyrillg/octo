# The octo project

cmake_minimum_required (VERSION 3.10)

project(astar_planner_viewer VERSION 1.0)


# ----------------------------------------------------------------------------
# CONFIGURE
# ----------------------------------------------------------------------------

include(cmake/standard_project_settings.cmake)

if (MSVC)
    add_compile_options(/W4 /WX)
else()
    add_compile_options(-Wall -Wextra -pedantic -Werror)
endif()

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

# set(gcc_like_cxx "$<COMPILE_LANG_AND_ID:CXX,ARMClang,AppleClang,Clang,GNU>")
# set(msvc_cxx "$<COMPILE_LANG_AND_ID:CXX,MSVC>")
# target_compile_options(${PROJECT_NAME}_options INTERFACE
#   "$<${gcc_like_cxx}:$<BUILD_INTERFACE:-Wall;-Wextra;-Wshadow;-Wformat=2;-Wunused>>"
#   "$<${msvc_cxx}:$<BUILD_INTERFACE:-W3>>"
# )

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")

#option(BUILD_SHARED_LIBS "Build using shared libraries" ON)

# ----------------------------------------------------------------------------
# BUILD
# ----------------------------------------------------------------------------

configure_file(src/${PROJECT_NAME}_config.hpp.in ${PROJECT_NAME}_config.hpp)

add_subdirectory(src)
list(APPEND EXTRA_LIBS math_functions)

# Add source to this project's executable.
add_executable (${PROJECT_NAME} src/main.cc)
target_link_libraries(${PROJECT_NAME} PUBLIC ${EXTRA_LIBS})
target_include_directories(${PROJECT_NAME} PUBLIC
                           ${PROJECT_BINARY_DIR}
                           )
#target_compile_definitions(${PROJECT_NAME}
#                           PRIVATE USELESS_VAR)


# ----------------------------------------------------------------------------
# INSTALL
# ----------------------------------------------------------------------------

install(TARGETS ${PROJECT_NAME} DESTINATION bin)
install(FILES ${PROJECT_BINARY_DIR}/${PROJECT_NAME}_config.hpp
  DESTINATION include
  )
message("PREFIX: ${CMAKE_INSTALL_PREFIX}")

install(EXPORT math_functionsTargets
  FILE math_functionsTargets.cmake
  DESTINATION lib/cmake/math_functions
)

include(CMakePackageConfigHelpers)
# generate the config file that includes the exports
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/Config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/math_functionsConfig.cmake
  INSTALL_DESTINATION "lib/cmake/example"
  NO_SET_AND_CHECK_MACRO
  NO_CHECK_REQUIRED_COMPONENTS_MACRO
  )
# generate the version file for the config file
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/math_functionsConfigVersion.cmake
  VERSION "${${PROJECT_NAME}_VERSION_MAJOR}.${${PROJECT_NAME}_VERSION_MINOR}"
  COMPATIBILITY AnyNewerVersion
)

# install the configuration file
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/math_functionsConfig.cmake
  DESTINATION lib/cmake/math_functions
  )

# allow use of the package within a project
export(EXPORT math_functionsTargets
  FILE ${CMAKE_CURRENT_BINARY_DIR}/math_functionsTargets.cmake
)

# ----------------------------------------------------------------------------
# INSTALLERS
# ----------------------------------------------------------------------------

include(InstallRequiredSystemLibraries)
#set(CPACK_GENERATOR "DEB")
set(CPACK_GENERATOR "WIX")
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE.txt")
set(CPACK_PACKAGE_VERSION_MAJOR "${${PROJECT_NAME}_VERSION_MAJOR}")
set(CPACK_PACKAGE_VERSION_MINOR "${${PROJECT_NAME}_VERSION_MINOR}")
SET(CPACK_DEBIAN_PACKAGE_MAINTAINER "CG")
include(CPack)
