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


# Prevent overriding the parent project's compiler/linker settings on Windows
set(gtest_force_shared_crt
    ON
    CACHE BOOL "" FORCE)

# Add googletest directly to our build. This defines the gtest and gtest_main
# targets.
if(NOT TARGET gtest)
    add_subdirectory(${GTEST_PATH} ${CMAKE_BINARY_DIR}/googletest-build
                     EXCLUDE_FROM_ALL)
endif()

# The gtest/gtest_main targets carry header search path dependencies
# automatically when using CMake 2.8.11 or later. Otherwise we have to add them
# here ourselves.
include_directories("${GTEST_PATH}/googletest/include")

set(TEST_DATA ${CMAKE_CURRENT_SOURCE_DIR}/data)

# Configure modules header with variables configured, last position to work!
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/common/config.h.in"
               "${CMAKE_CURRENT_SOURCE_DIR}/common/config.h")

add_subdirectory(${PROJ_MAIN_NAME}_tests)
add_subdirectory(mem_${PROJ_MAIN_NAME}_tests)
if(KPSR_WITH_YAML)
    add_subdirectory(yaml_${PROJ_MAIN_NAME}_tests)
endif()

if(KPSR_WITH_ZMQ)
    add_subdirectory(zmq_${PROJ_MAIN_NAME}_tests)
endif()
if(KPSR_WITH_DDS)
    add_subdirectory(dds_${PROJ_MAIN_NAME}_tests)
endif()
if(KPSR_TEST_PERFORMANCE)
    add_subdirectory(performance)
endif()
