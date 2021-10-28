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

SET(PROJ_NAME kpsr_socket_${PROJ_MAIN_NAME}_example)
PROJECT(${PROJ_NAME})

# Source
#---------------------------------------------------#
FILE(GLOB ${PROJ_NAME}_SRC
    "src/*.cpp"
)

# Create Library
#---------------------------------------------------#
ADD_EXECUTABLE(${PROJ_NAME}_server ${${PROJ_NAME}_HEADERS} src/server.cpp)
ADD_EXECUTABLE(${PROJ_NAME}_client ${${PROJ_NAME}_HEADERS} src/client.cpp)

# Link libraries with Project
#---------------------------------------------------#
TARGET_LINK_LIBRARIES(${PROJ_NAME}_server kpsr_socket_core)
TARGET_LINK_LIBRARIES(${PROJ_NAME}_client kpsr_socket_core)

TARGET_INCLUDE_DIRECTORIES(${PROJ_NAME}_server
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
TARGET_INCLUDE_DIRECTORIES(${PROJ_NAME}_client
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
