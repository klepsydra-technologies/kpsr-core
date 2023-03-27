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

set(PROJ_NAME kpsr_socket_${PROJ_MAIN_NAME}_example)
project(${PROJ_NAME})

# Source
# ---------------------------------------------------#
file(GLOB ${PROJ_NAME}_SRC "src/*.cpp")

# Create Library
# ---------------------------------------------------#
add_executable(${PROJ_NAME}_server ${${PROJ_NAME}_HEADERS} src/server.cpp)
add_executable(${PROJ_NAME}_client ${${PROJ_NAME}_HEADERS} src/client.cpp)

# Link libraries with Project
# ---------------------------------------------------#
target_link_libraries(${PROJ_NAME}_server kpsr_socket_core)
target_link_libraries(${PROJ_NAME}_client kpsr_socket_core)

target_include_directories(${PROJ_NAME}_server
                           PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(${PROJ_NAME}_client
                           PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
