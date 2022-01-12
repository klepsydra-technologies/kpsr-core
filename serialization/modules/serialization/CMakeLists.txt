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

set(PROJ_NAME kpsr_${PROJ_MAIN_NAME})
project(${PROJ_NAME})

# Source
# ---------------------------------------------------#
file(GLOB_RECURSE ${PROJ_NAME}_HEADERS "include/*.h")

# Create Library
# ---------------------------------------------------#
add_library(${PROJ_NAME} INTERFACE)
target_include_directories(
    ${PROJ_NAME}
    INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
              $<INSTALL_INTERFACE:include>
              $<BUILD_INTERFACE:${THIRDPARTIES_PATH}/include>)

# # Link libraries with Project
# #---------------------------------------------------#
# TARGET_LINK_LIBRARIES(${PROJ_NAME})

# Install
# ---------------------------------------------------#
install(
    TARGETS ${PROJ_NAME}
    EXPORT KlepsydraTargets
    PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
    ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    RUNTIME DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT libs)

install(
    DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/klepsydra"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
    COMPONENT dev)

add_core_export_target(${PROJ_NAME})

# PRINTBASICINFO(${PROJ_NAME})
