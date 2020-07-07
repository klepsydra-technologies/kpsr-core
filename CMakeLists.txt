#****************************************************************************
#
#                           Klepsydra Core Modules
#              Copyright (C) 2019-2020  Klepsydra Technologies GmbH
#                            All Rights Reserved.
#
#  This file is subject to the terms and conditions defined in
#  file 'LICENSE.md', which is part of this source code package.
#
#  NOTICE:  All information contained herein is, and remains the property of Klepsydra
#  Technologies GmbH and its suppliers, if any. The intellectual and technical concepts
#  contained herein are proprietary to Klepsydra Technologies GmbH and its suppliers and
#  may be covered by Swiss and Foreign Patents, patents in process, and are protected by
#  trade secret or copyright law. Dissemination of this information or reproduction of
#  this material is strictly forbidden unless prior written permission is obtained from
#  Klepsydra Technologies GmbH.
#
#****************************************************************************

# Main header: Project name & Configuration
# ------------------------------------------------------------------------------------------------------
CMAKE_MINIMUM_REQUIRED(VERSION 2.8.11)

SET(PROJ_MAIN_NAME "klepsydra")
PROJECT(${PROJ_MAIN_NAME})

set(KLEPSYDRA_MAJOR_VERSION 2)
set(KLEPSYDRA_MINOR_VERSION 1)
set(KLEPSYDRA_PATCH_VERSION 0)
set(KLEPSYDRA_VERSION
  ${KLEPSYDRA_MAJOR_VERSION}.${KLEPSYDRA_MINOR_VERSION}.${KLEPSYDRA_PATCH_VERSION})

# Configuration types
SET(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "Configs" FORCE)
IF(DEFINED CMAKE_BUILD_TYPE AND CMAKE_VERSION VERSION_GREATER "2.8")
  SET_PROPERTY(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS ${CMAKE_CONFIGURATION_TYPES})
ENDIF()

#Install binaries
IF(DEFINED THIRDPARTIES_PATH)
ELSE()
  SET(THIRDPARTIES_PATH ${CMAKE_SOURCE_DIR}/thirdparties )
ENDIF()
message(STATUS "Thirdparties install path: " ${THIRDPARTIES_PATH})

IF(DEFINED GTEST_PATH)
ELSE()
        SET(GTEST_PATH ${THIRDPARTIES_PATH}/googletest) #default google test path
ENDIF()
message(STATUS "Google test path: " ${GTEST_PATH})
if(DEFINED KPSR_BUILD_PATH)
else()
    SET(KPSR_BUILD_PATH ${CMAKE_SOURCE_DIR}/kpsr-build)
endif()
message(STATUS "kpsr-build path: " ${KPSR_BUILD_PATH})

# Configure variables
# ------------------------------------------------------------------------------------------------------

# Paths
SET(CMAKE_MODULE_PATH            ${KPSR_BUILD_PATH}/cmake)
SET(LIBRARY_OUTPUT_PATH          ${PROJECT_BINARY_DIR}/bin                   CACHE PATH "Library output" FORCE)
SET(EXECUTABLE_OUTPUT_PATH       ${PROJECT_BINARY_DIR}/bin                   CACHE PATH "Executable output" FORCE)

# Must use GNUInstallDirs to install libraries into correct
# locations on all platforms.
include(GNUInstallDirs)

# Build type
SET(BUILD_SHARED_LIBS "ON" FORCE)

include(CheckCXXCompilerFlag)
include(CheckIncludeFileCXX)

check_cxx_compiler_flag(-std=gnu++11 HAS_CXX11)
if (HAS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
endif()

if (NOT APPLE)
    check_cxx_compiler_flag(-Wl,--no-undefined HAS_NO_UNDEFINED)
    if (HAS_NO_UNDEFINED)
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--no-undefined")
        set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--no-undefined")
        set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--no-undefined")
    endif()
endif()

check_cxx_compiler_flag(-pedantic HAS_PEDANTIC)
if (HAS_PEDANTIC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pedantic")
endif()

check_cxx_compiler_flag(-Wall HAS_WALL)
if (HAS_WALL)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
endif()

check_include_file_cxx(sys/epoll.h HAS_EPOLL)
if (HAS_EPOLL)
    add_definitions(-DHAS_EPOLL)
else()
    add_definitions(-DUSE_GET_WRITE_QUEUE)
endif()

enable_testing()
if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    # require at least gcc 4.8
    if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS 7.2)
    else()
        add_definitions(-faligned-new)
    endif()
endif()

add_definitions(-fno-strict-overflow)
if (KPSR_WITH_ZMQ)
   add_definitions(-Wno-deprecated-declarations)
   add_definitions(-Wno-comment)
ENDIF()

if (KPSR_WITH_CODE_METRICS)
   # Code Coverage
   IF (CMAKE_BUILD_TYPE MATCHES Debug)
       add_definitions(--coverage)
       SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -fprofile-arcs -ftest-coverage")
       SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -O0 -fprofile-arcs -ftest-coverage")
       include(FindPythonInterp)
       include(CodeCoverage)
       setup_target_for_all_tests_coverage()
       setup_target_for_all_tests_coverage_cobertura(coverage)
   ENDIF()

   # Code style check
   include(CppcheckTargets)
   include(Findcppcheck)
ENDIF()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

include(KlepsydraBuild)

# Configuration current project
# ------------------------------------------------------------------------------------------------------

#Dependencies

find_package(Threads REQUIRED)

# Project Logic
# ------------------------------------------------------------------------------------------------------

######################################################################################
# ADD SUB FOLDERS
######################################################################################
ADD_SUBDIRECTORY(thirdparties)
ADD_SUBDIRECTORY(serialization)
ADD_SUBDIRECTORY(core)
ADD_SUBDIRECTORY(high_performance)
ADD_SUBDIRECTORY(state_machine)
if (KPSR_RUNTIME)
else()
   ADD_SUBDIRECTORY(code_generator)
ENDIF()
IF(KPSR_WITH_NODE)
    ADD_SUBDIRECTORY(node)
ENDIF()

if (KPSR_RUNTIME)
else()
   # add a target to generate API documentation with Doxygen
   if(KPSR_WITH_DOXYGEN)
       find_package(Doxygen REQUIRED)
       set(DOC_PROJECT_NAME "Klepsydra Core")
       set(DOC_PROJECT_BRIEF "Klepsydra Core API")

       # Generate documentation only for directories where there is a group_definitions.h file
       MACRO(PYTHON_DIRECTORIES return_list)
           FILE(GLOB_RECURSE new_list "code_generator/*.py")
           SET(dir_list "")
           FOREACH(file_path ${new_list})
               GET_FILENAME_COMPONENT(dir_path ${file_path} PATH)
               SET(dir_list ${dir_list} ${dir_path})
           ENDFOREACH()
           LIST(REMOVE_DUPLICATES dir_list)
           SET(${return_list} ${dir_list})
       ENDMACRO()
       PYTHON_DIRECTORIES(PY_DOC_DIRS)
       string(REGEX REPLACE ";" " " PY_DOC_DIRS "${PY_DOC_DIRS}")

       file(GLOB_RECURSE DOC_DIRS "*/modules/group_definitions.h")
       string(REGEX REPLACE "/group_definitions.h[;]?" " " DOC_DIRS "${DOC_DIRS}")
       list (APPEND DOC_DIRS ${PY_DOC_DIRS})

       configure_file(${KPSR_BUILD_PATH}/docs/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
       add_custom_target(doc
           ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
           COMMAND moxygen --templates ${KPSR_BUILD_PATH}/docs/moxygen/templates --groups --output api-%s.md ./xml
           WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
           COMMENT "Generating API documentation with Doxygen" VERBATIM
       )
   endif(KPSR_WITH_DOXYGEN)
ENDIF()


set(DEF_INSTALL_CMAKE_DIR lib/cmake/klepsydra)
set(INSTALL_CMAKE_DIR ${DEF_INSTALL_CMAKE_DIR} CACHE PATH "Installation directory for CMake files")

# Make relative paths absolute (needed later on)
foreach(p LIB BIN INCLUDE CMAKE)
  set(var INSTALL_${p}_DIR)
  if(NOT IS_ABSOLUTE "${${var}}")
    set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
  endif()
endforeach()

list (APPEND EXPORT_INCLUDE_DIRS ${CMAKE_INSTALL_FULL_INCLUDEDIR})
message ("CORE_EXPORT_TARGETS: " ${CORE_EXPORT_TARGETS})
message ("DDS_EXPORT_TARGETS: " ${DDS_EXPORT_TARGETS})
message ("ZMQ_EXPORT_TARGETS: " ${ZMQ_EXPORT_TARGETS})
message ("EXPORT_INCLUDE_DIRS: " ${EXPORT_INCLUDE_DIRS})
export(TARGETS ${CORE_EXPORT_TARGETS} ${DDS_EXPORT_TARGETS} ${ZMQ_EXPORT_TARGETS} FILE "${PROJECT_BINARY_DIR}/KlepsydraTargets.cmake")

# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE klepsydra)

# Create the KlepsydraConfig.cmake and KlepsydraConfigVersion files
configure_file(KlepsydraConfig.cmake.in
  "${PROJECT_BINARY_DIR}/KlepsydraConfig.cmake" @ONLY)

configure_file(KlepsydraConfig.cmake.in
  "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/KlepsydraConfig.cmake" @ONLY)
# ... for both
configure_file(KlepsydraConfigVersion.cmake.in
  "${PROJECT_BINARY_DIR}/KlepsydraConfigVersion.cmake" @ONLY)

configure_file(${CMAKE_SOURCE_DIR}/.githooks/pre-commit.in
  "${CMAKE_SOURCE_DIR}/.git/hooks/pre-commit" @ONLY)

# Install the KlepsydraConfig.cmake and KlepsydraConfigVersion.cmake
install(FILES
  "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/KlepsydraConfig.cmake"
  "${PROJECT_BINARY_DIR}/KlepsydraConfigVersion.cmake"
  "${CMAKE_SOURCE_DIR}/code_generator/cmake/KpsrCodeGen.cmake"
  DESTINATION "${INSTALL_CMAKE_DIR}" COMPONENT dev)

# Install the export set for use with the install-tree
install(EXPORT KlepsydraTargets DESTINATION
  "${INSTALL_CMAKE_DIR}" COMPONENT dev)

