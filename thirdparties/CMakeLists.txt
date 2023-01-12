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


SET(PROJ_MAIN_NAME "thirdparties")
PROJECT(${PROJ_MAIN_NAME})

file(COPY googletest DESTINATION ${THIRDPARTIES_PATH})
file(COPY cereal/include DESTINATION ${THIRDPARTIES_PATH})
file(COPY spdlog/include DESTINATION ${THIRDPARTIES_PATH})

file(GLOB CONCURRENTQUEUE_HEADERS
  "concurrentqueue/*.h"
)
file(COPY ${CONCURRENTQUEUE_HEADERS} DESTINATION ${THIRDPARTIES_PATH}/include)

# Install
#---------------------------------------------------#
INSTALL(DIRECTORY "${THIRDPARTIES_PATH}/include"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/kpsr_3parties"
)

add_include_dirs("${CMAKE_INSTALL_FULL_INCLUDEDIR}/kpsr_3parties/include")
