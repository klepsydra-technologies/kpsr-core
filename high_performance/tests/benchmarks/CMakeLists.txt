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


set(PROJ_NAME kpsr_${PROJ_MAIN_NAME}_benchmarks)
project(${PROJ_NAME})

find_package(benchmark REQUIRED)

# Source
# ---------------------------------------------------#
file(GLOB_RECURSE ${PROJ_NAME}_HEADERS "include/*.h")

file(GLOB_RECURSE ${PROJ_NAME}_SRC "src/*.cpp")

add_executable(${PROJ_NAME} ${${PROJ_NAME}_HEADERS} ${${PROJ_NAME}_SRC})

target_link_libraries(${PROJ_NAME} kpsr_high_performance
                      benchmark::benchmark_main)

target_include_directories(${PROJ_NAME}
                           PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
