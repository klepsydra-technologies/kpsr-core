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

set(PROJ_NAME kpsr_${PROJ_MAIN_NAME}_tests)
project(${PROJ_NAME})

# Include directories with headers
# ---------------------------------------------------#
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(
    ${CMAKE_SOURCE_DIR}/${PROJ_MAIN_NAME}/modules/${PROJ_MAIN_NAME}/include)
include_directories(
    ${CMAKE_SOURCE_DIR}/${PROJ_MAIN_NAME}/modules/json_conf/include)

# Source
# ---------------------------------------------------#
file(GLOB ${PROJ_NAME}_SRC "src/*.cpp")

# Create Library
# ---------------------------------------------------#
add_executable(${PROJ_NAME} ${${PROJ_NAME}_HEADERS} ${${PROJ_NAME}_SRC})

# Link libraries with Project
# ---------------------------------------------------#
target_link_libraries(${PROJ_NAME} kpsr_state_machine kpsr_state_machine_json
                      gtest_main)

add_test(NAME ${PROJ_NAME} COMMAND ${PROJ_NAME} --output-on-failure
                                   --gtest_output=xml:gtestresults.xml)
