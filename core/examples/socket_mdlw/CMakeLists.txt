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
