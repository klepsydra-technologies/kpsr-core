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

SET(PROJ_MAIN_NAME "codegen")
PROJECT(${PROJ_MAIN_NAME})

# Configuration types
SET(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "Configs" FORCE)
IF(DEFINED CMAKE_BUILD_TYPE AND CMAKE_VERSION VERSION_GREATER "2.8")
  SET_PROPERTY(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS ${CMAKE_CONFIGURATION_TYPES})
ENDIF()

#Install binaries
IF(DEFINED THIRDPARTIES_PATH)
ELSE()
  SET(THIRDPARTIES_PATH ${CMAKE_SOURCE_DIR}/../../thirdparties )
ENDIF()
message(STATUS "Thirdparties install path: " ${THIRDPARTIES_PATH})

IF(DEFINED GTEST_PATH)
ELSE()
        SET(GTEST_PATH ${THIRDPARTIES_PATH}/googletest) #default google test path
ENDIF()
message(STATUS "Google test path: " ${GTEST_PATH})
if(DEFINED KPSR_BUILD_PATH)
else()
	SET(KPSR_BUILD_PATH ${CMAKE_SOURCE_DIR}/../../kpsr-build)
endif()
message(STATUS "kpsr-build path: " ${KPSR_BUILD_PATH})

# Configure variables
# ------------------------------------------------------------------------------------------------------

# Paths
SET(CMAKE_MODULE_PATH            ${KPSR_BUILD_PATH}/cmake ${CMAKE_SOURCE_DIR}/../cmake)
SET(LIBRARY_OUTPUT_PATH          ${PROJECT_BINARY_DIR}/bin                   CACHE PATH "Library output" FORCE)
SET(EXECUTABLE_OUTPUT_PATH       ${PROJECT_BINARY_DIR}/bin                   CACHE PATH "Executable output" FORCE)

# Must use GNUInstallDirs to install libraries into correct
# locations on all platforms.
include(GNUInstallDirs)

# Build type
SET(BUILD_SHARED_LIBS "ON" FORCE)

include(KpsrCodeGen)

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

add_definitions(-std=c++11)
enable_testing()
if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    # require at least gcc 4.8
    if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS 7.5)
    else()
        add_definitions(-faligned-new)
    endif()
endif()

add_definitions(-fno-strict-overflow)
if (KPSR_WITH_ZMQ)
   add_definitions(-Wno-deprecated-declarations)
   add_definitions(-Wno-comment)
ENDIF()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# Configuration current project
# ------------------------------------------------------------------------------------------------------

#Dependencies

find_package(Threads REQUIRED)

# Project Logic
# ------------------------------------------------------------------------------------------------------

######################################################################################
# ADD SUB FOLDERS
######################################################################################
ADD_SUBDIRECTORY(modules)
ADD_SUBDIRECTORY(tests)

