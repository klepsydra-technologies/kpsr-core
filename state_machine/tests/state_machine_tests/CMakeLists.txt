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

set(PROJ_NAME kpsr_${PROJ_MAIN_NAME}_tests)
project(${PROJ_NAME})

# Include directories with headers
# ---------------------------------------------------#
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(
    ${CMAKE_SOURCE_DIR}/${PROJ_MAIN_NAME}/modules/${PROJ_MAIN_NAME}/include)
include_directories(
    ${CMAKE_SOURCE_DIR}/${PROJ_MAIN_NAME}/modules/yaml_conf/include)

# Source
# ---------------------------------------------------#
file(GLOB ${PROJ_NAME}_SRC "src/*.cpp")

# Create Library
# ---------------------------------------------------#
add_executable(${PROJ_NAME} ${${PROJ_NAME}_HEADERS} ${${PROJ_NAME}_SRC})

# Link libraries with Project
# ---------------------------------------------------#
target_link_libraries(${PROJ_NAME} kpsr_state_machine yaml-cpp
                      kpsr_state_machine_yaml gtest_main)

add_test(NAME ${PROJ_NAME} COMMAND ${PROJ_NAME} --output-on-failure
                                   --gtest_output=xml:gtestresults.xml)
