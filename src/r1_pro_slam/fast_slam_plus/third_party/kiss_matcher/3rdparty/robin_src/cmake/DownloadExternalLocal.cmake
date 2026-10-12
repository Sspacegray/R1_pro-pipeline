# Local, network-free replacements for robin's DownloadExternal.cmake
# All external sources are vendored under extern/ so colcon builds never
# git-clone at configure time.
macro(find_external_dependency PACKAGE_NAME TARGET_NAME INCLUDED_CMAKE_PATH)
  string(TOUPPER ${PACKAGE_NAME} PACKAGE_NAME_UP)
  set(USE_FROM_SYSTEM_OPTION "USE_SYSTEM_${PACKAGE_NAME_UP}")
  if(${${USE_FROM_SYSTEM_OPTION}})
    find_package(${PACKAGE_NAME} QUIET NO_MODULE)
  endif()
  if(NOT ${USE_FROM_SYSTEM_OPTION} OR NOT TARGET ${TARGET_NAME})
    find_package(${PACKAGE_NAME} REQUIRED NO_MODULE)
  endif()
endmacro()

macro(_local_download_project)
  cmake_parse_arguments(LP "" "PROJ" "" ${ARGN})
  set(${LP_PROJ}_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/extern/${LP_PROJ}-src")
  set(${LP_PROJ}_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/${LP_PROJ}-build")
endmacro()

macro(download_project)
  _local_download_project(${ARGN})
endmacro()

macro(robin_download_pybind11)
  _local_download_project(PROJ pybind11)
endmacro()

macro(robin_download_pmc)
  _local_download_project(PROJ pmc)
endmacro()

macro(robin_download_atomic_queue)
  _local_download_project(PROJ atomicQueue)
endmacro()

macro(robin_download_xenium)
  _local_download_project(PROJ xenium)
  if(NOT TARGET xenium)
    add_library(xenium INTERFACE)
    target_include_directories(xenium INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/extern/xenium-src>)
  endif()
endmacro()
