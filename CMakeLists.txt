# Copyright 2023 Klepsydra Technologies AG
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Main header: Project name & Configuration
# ------------------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.12)

if(DEFINED KPSR_BUILD_PATH)

else()
    set(KPSR_BUILD_PATH ${CMAKE_SOURCE_DIR}/kpsr-build)
endif()

message(STATUS "kpsr-build path: " ${KPSR_BUILD_PATH})

if(DEFINED THIRDPARTIES_PATH)

else()
    set(THIRDPARTIES_PATH ${CMAKE_SOURCE_DIR}/thirdparties)
endif()

message(STATUS "Thirdparties install path: " ${THIRDPARTIES_PATH})

# Paths
set(CMAKE_MODULE_PATH ${KPSR_BUILD_PATH}/cmake)

# Including CMake modules
# -----------------------
# Common modules
# ----------
# Must use GNUInstallDirs to install libraries into correct locations on all
# platforms.
include(GNUInstallDirs)
include(CheckCXXCompilerFlag)
include(CheckIncludeFileCXX)
include(GenerateVersion)
include(SetCxxStandard)
include(KlepsydraBuild)
include(KlepsydraLinkerFlags)
include(ConfigureLinux)

set(PROJ_MAIN_NAME "kpsr-core")
project(${PROJ_MAIN_NAME})

if(NOT DEFINED GTEST_PATH)
    set(GTEST_PATH ${THIRDPARTIES_PATH}/googletest) # default google test
    # path
endif()
message(STATUS "Google test path: " ${GTEST_PATH})

# Configuration types
set(CMAKE_CONFIGURATION_TYPES
    "Debug;Release"
    CACHE STRING "Configs" FORCE)

if(DEFINED CMAKE_BUILD_TYPE AND CMAKE_VERSION VERSION_GREATER "2.8")
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
                                                 ${CMAKE_CONFIGURATION_TYPES})
endif()

# Configure variables
# ------------------------------------------------------------------------------------------------------
set(LIBRARY_OUTPUT_PATH
    ${PROJECT_BINARY_DIR}/bin
    CACHE PATH "Library output" FORCE)
set(EXECUTABLE_OUTPUT_PATH
    ${PROJECT_BINARY_DIR}/bin
    CACHE PATH "Executable output" FORCE)

set(KPSR_VERSION_HEADER_FILE
    "${CMAKE_CURRENT_SOURCE_DIR}/core/modules/core/src/version.cpp")
get_kpsr_version(KLEPSYDRA_CORE_VERSION "${KPSR_VERSION_HEADER_FILE}")
message(STATUS "Klepsydra core version is: ${KLEPSYDRA_CORE_VERSION}")

set(MINIMAL_BUILD
    Off
    CACHE BOOL "Minimal build mode")

include(KlepsydraExamplesTests)

set(BUILD_SHARED_LIBS
    ON
    CACHE BOOL "Build shared libraries")

kpsr_set_linker_flags()

check_cxx_compiler_flag(-pedantic HAS_PEDANTIC)

if(HAS_PEDANTIC)
    add_compile_options($<$<COMPILE_LANGUAGE:C,CXX>:-pedantic>)
endif()

check_cxx_compiler_flag(-Wall HAS_WALL)

if(HAS_WALL)
    add_compile_options($<$<COMPILE_LANGUAGE:C,CXX>:-Wall>)
endif()

check_include_file_cxx(sys/epoll.h HAS_EPOLL)

if(HAS_EPOLL)
    add_compile_definitions($<$<COMPILE_LANGUAGE:CXX>:HAS_EPOLL>)
else()
    add_compile_definitions($<$<COMPILE_LANGUAGE:CXX>:USE_GET_WRITE_QUEUE>)
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    # require at least gcc 4.8
    if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 7.5)
        add_compile_options($<$<COMPILE_LANGUAGE:C,CXX>:-faligned-new>)
    endif()
endif()

add_compile_options($<$<COMPILE_LANGUAGE:C,CXX>:-fno-strict-overflow>)

if(KPSR_WITH_ZMQ)
    add_compile_options(
        $<$<COMPILE_LANGUAGE:C,CXX>:-Wno-deprecated-declarations>)
    add_compile_options($<$<COMPILE_LANGUAGE:C,CXX>:-Wno-comment>)
    string(APPEND KPSR_PACKAGE_OPTIONS "-zmq")
endif()

set(_SPDLOG_VERSION
    "1.8.5"
    CACHE STRING "Spdlog version to look for" FORCE)
set(_SPDLOG_OPTIONS "QUIET")

find_package(spdlog ${_SPDLOG_VERSION} ${_SPDLOG_OPTIONS})

if(KPSR_WITH_CODE_METRICS)
    if(CMAKE_BUILD_TYPE MATCHES Debug AND CMAKE_COMPILER_IS_GNUCXX)
        include(CodeCoverage)
        set(CODE_COVERAGE_VERBOSE ON)
        append_coverage_compiler_flags()
    endif()

    # Code style check
    include(CppcheckTargets)
    include(Findcppcheck)
endif()

# Code format check
include(CodeFormat)

# CCACHE support
include(FindCcache)

if(DEFINED KPSR_WITH_MARCH)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=${KPSR_WITH_MARCH}")
endif()

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

if(KPSR_ENABLE_TESTS)
    enable_testing()
endif()

# Configuration current project
# ------------------------------------------------------------------------------------------------------

# Dependencies

find_package(Threads REQUIRED)

# Project Logic
# ------------------------------------------------------------------------------------------------------

# ##############################################################################
# ADD SUB FOLDERS
# ##############################################################################
find_package(KlepsydraSDK 2 REQUIRED)
# TODO: DEFINE THIS MACRO IN KPSR-BUILD WHICH RECEIVES A LIST OF DEPENDENCIES
set(CPACK_DEBIAN_LIBS_PACKAGE_DEPENDS "kpsr-sdk${PACKAGE_SUFFIX} (<< 3)")
set(CPACK_DEBIAN_DEV_PACKAGE_DEPENDS "kpsr-sdk${PACKAGE_SUFFIX}-dev (<< 3)")
set(KPSR_CHECK_DEPS
    "include(CMakeFindDependencyMacro)
find_dependency(KlepsydraSDK 2)
")

add_subdirectory(thirdparties)
add_subdirectory(serialization)
add_subdirectory(core)
add_subdirectory(high_performance)
add_subdirectory(state_machine)

if(KPSR_TEST_PERFORMANCE)
    add_subdirectory(benchmarks)
endif()

# Code Coverage
if(KPSR_WITH_CODE_METRICS
   AND CMAKE_BUILD_TYPE MATCHES Debug
   AND CMAKE_COMPILER_IS_GNUCXX)
    string(REPLACE ${CMAKE_SOURCE_DIR}/ "" THIRDPARTIES_PATH_COVERAGE
                   ${THIRDPARTIES_PATH})
    list(
        APPEND
        COVERAGE_EXCLUDES
        ${THIRDPARTIES_PATH_COVERAGE}
        ${CMAKE_PREFIX_PATH}
        core/tests
        core/examples
        high_performance/tests
        serialization/tests
        /usr/
        /opt)
    setup_target_for_coverage_gcovr_xml(NAME coverage EXECUTABLE make test)
endif()

if(KPSR_RUNTIME)

else()
    # add a target to generate API documentation with Doxygen
    if(KPSR_WITH_DOXYGEN)
        find_package(Doxygen REQUIRED)
        set(DOC_PROJECT_NAME "Klepsydra Core")
        set(DOC_PROJECT_BRIEF "Klepsydra Core API")

        # Generate documentation only for directories where there is a
        # group_definitions.h file
        macro(PYTHON_DIRECTORIES return_list)
            file(GLOB_RECURSE new_list "code_generator/*.py")
            set(dir_list "")

            foreach(file_path ${new_list})
                get_filename_component(dir_path ${file_path} PATH)
                set(dir_list ${dir_list} ${dir_path})
            endforeach()

            list(REMOVE_DUPLICATES dir_list)
            set(${return_list} ${dir_list})
        endmacro()

        python_directories(PY_DOC_DIRS)
        string(REGEX REPLACE ";" " " PY_DOC_DIRS "${PY_DOC_DIRS}")

        file(GLOB_RECURSE DOC_DIRS "*/modules/group_definitions.h")
        string(REGEX REPLACE "/group_definitions.h[;]?" " " DOC_DIRS
                             "${DOC_DIRS}")
        list(APPEND DOC_DIRS ${PY_DOC_DIRS})

        configure_file(${KPSR_BUILD_PATH}/docs/Doxyfile.in
                       ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
        add_custom_target(
            doc
            ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
            COMMAND
                moxygen --templates ${KPSR_BUILD_PATH}/docs/moxygen/templates
                --groups --output api-%s.md ./xml
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            COMMENT "Generating API documentation with Doxygen"
            VERBATIM)
    endif(KPSR_WITH_DOXYGEN)
endif()

set(INSTALL_CMAKE_DIR ${CMAKE_INSTALL_LIBDIR}/cmake/klepsydra)

list(APPEND EXPORT_INCLUDE_DIRS ${CMAKE_INSTALL_FULL_INCLUDEDIR})
message("CORE_EXPORT_TARGETS: " ${CORE_EXPORT_TARGETS})
message("ZMQ_EXPORT_TARGETS: " ${ZMQ_EXPORT_TARGETS})
message("EXPORT_INCLUDE_DIRS: " ${EXPORT_INCLUDE_DIRS})
export(TARGETS ${CORE_EXPORT_TARGETS} ${ZMQ_EXPORT_TARGETS}
       FILE "${PROJECT_BINARY_DIR}/KlepsydraTargets.cmake")

# Install binaries

# Export the package for use from the build-tree (this registers the build-tree
# with a global CMake-registry)
export(PACKAGE klepsydra)

if(KPSR_WITH_ZMQ)
    set(KPSR_ZMQ_DEP "find_dependency(cppzmq 4.3 QUIET)
")
    string(APPEND KPSR_CHECK_DEPS ${KPSR_ZMQ_DEP})
endif()

if(KPSR_COMPILE_SPDLOG)
    set(KPSR_SPDLOG_DEP "find_dependency(spdlog)
")
    string(APPEND KPSR_CHECK_DEPS ${KPSR_SPDLOG_DEP})
endif()

# Create the KlepsydraConfig.cmake and KlepsydraConfigVersion files
include(CMakePackageConfigHelpers)

configure_package_config_file(
    KlepsydraConfig.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/KlepsydraConfig.cmake
    INSTALL_DESTINATION ${INSTALL_CMAKE_DIR}
    PATH_VARS CMAKE_INSTALL_INCLUDEDIR CMAKE_INSTALL_LIBDIR)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/KlepsydraConfigVersion.cmake
    VERSION ${KLEPSYDRA_CORE_VERSION}
    COMPATIBILITY SameMajorVersion)

install(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/KlepsydraConfig.cmake
          ${CMAKE_CURRENT_BINARY_DIR}/KlepsydraConfigVersion.cmake
    DESTINATION ${INSTALL_CMAKE_DIR}
    COMPONENT dev)

# Install the export set for use with the install-tree
install(
    EXPORT KlepsydraTargets
    FILE KlepsydraTargets.cmake
    DESTINATION "${INSTALL_CMAKE_DIR}"
    COMPONENT dev)

# PACKAGING
if(NOT DEFINED KPSR_VERSION_MAJOR)
    set(KPSR_VERSION_MAJOR ${KLEPSYDRA_CORE_VERSION_MAJOR})
endif()

if(NOT DEFINED KPSR_VERSION_MINOR)
    set(KPSR_VERSION_MINOR ${KLEPSYDRA_CORE_VERSION_MINOR})
endif()

if(NOT DEFINED KPSR_VERSION_PATCH)
    set(KPSR_VERSION_PATCH ${KLEPSYDRA_CORE_VERSION_PATCH})
endif()

set(KPSR_PACKAGE_DESCRIPTION "Klepsydra SDK Core Libraries")
set(KPSR_PACKAGE_DESCRIPTION_SUMMARY "Klepsydra SDK Core Libraries")

include(KlepsydraPackaging)
