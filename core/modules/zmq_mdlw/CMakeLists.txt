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

set(PROJ_NAME kpsr_zmq_${PROJ_MAIN_NAME})
project(${PROJ_NAME})

# find cppzmq wrapper, installed by make of cppzmq
find_package(cppzmq 4.3 REQUIRED)
# Include directories with headers
# ---------------------------------------------------#

# Source
# ---------------------------------------------------#
file(GLOB ${PROJ_NAME}_SRC "src/*.cpp")
file(GLOB_RECURSE ${PROJ_NAME}_HEADERS "include/*.h")

# Create Library
# ---------------------------------------------------#
add_library(${PROJ_NAME} ${${PROJ_MAIN_NAME}_LIB_TYPE} ${${PROJ_NAME}_HEADERS}
                         ${${PROJ_NAME}_SRC})

# Link libraries with Project
# ---------------------------------------------------#
target_link_libraries(${PROJ_NAME} PUBLIC kpsr_core kpsr_core_yaml cppzmq
                                          ${CMAKE_THREAD_LIBS_INIT})

target_include_directories(
    ${PROJ_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                        $<INSTALL_INTERFACE:include>)

set_target_properties(
    ${PROJ_NAME} PROPERTIES VERSION ${KLEPSYDRA_CORE_VERSION}
                            SOVERSION ${KLEPSYDRA_CORE_VERSION_MAJOR})

if(KPSR_WITH_CODE_METRICS)
    add_cppcheck(${PROJ_NAME} UNUSED_FUNCTIONS STYLE POSSIBLE_ERROR)
endif()

# Install
# ---------------------------------------------------#
install(
    TARGETS ${PROJ_NAME}
    EXPORT KlepsydraTargets
    DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    COMPONENT libs)
install(
    DIRECTORY "include/klepsydra"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
    COMPONENT dev)

add_zmq_export_target(${PROJ_NAME})

# PRINTBASICINFO(${PROJ_NAME})
