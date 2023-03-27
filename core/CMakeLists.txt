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

set(PROJ_MAIN_NAME "core")
project(${PROJ_MAIN_NAME})

# ##############################################################################
# ADD SUB FOLDERS
# ##############################################################################
add_subdirectory(modules)
if(KPSR_ENABLE_EXAMPLES)
    add_subdirectory(examples)
endif()
if(KPSR_ENABLE_TESTS)
    add_subdirectory(tests)
endif()

list(APPEND COVERAGE_EXCLUDES ${CMAKE_CURRENT_SOURCE_DIR}/tests
     ${CMAKE_CURRENT_SOURCE_DIR}/examples)
set(COVERAGE_EXCLUDES
    ${COVERAGE_EXCLUDES}
    PARENT_SCOPE)
add_core_export_target()
add_dds_export_target()
add_zmq_export_target()
add_include_dirs()
