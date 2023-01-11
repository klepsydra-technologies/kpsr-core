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

set(PROJ_NAME kpsr_dds_${PROJ_MAIN_NAME})
project(${PROJ_NAME})

file(REMOVE_RECURSE ${CMAKE_CURRENT_SOURCE_DIR}/gen)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/gen)

file(GLOB idls "./idl/*.idl")

foreach(idl ${idls})
    connext_idlgen(${idl} ${CMAKE_CURRENT_SOURCE_DIR})
    get_filename_component(nfile ${idl} NAME)
    connext_gendatamodel(${nfile} ${CMAKE_CURRENT_SOURCE_DIR})
endforeach(idl)

# Source
# ---------------------------------------------------#
file(GLOB_RECURSE ${PROJ_NAME}_HEADERS "include/*.h")
file(GLOB ${PROJ_NAME}_GEN_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/gen/*.h*")

# Create Library
# ---------------------------------------------------#
add_library(kpsr_dds_${PROJ_MAIN_NAME}_datamodel SHARED ${Connext_DATAMODEL})

# Link libraries with Project
# ---------------------------------------------------#
target_link_libraries(kpsr_dds_${PROJ_MAIN_NAME}_datamodel
                      PUBLIC RTIConnextDDS::cpp2_api kpsr_serialization)
target_include_directories(
    kpsr_dds_${PROJ_MAIN_NAME}_datamodel
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/gen>
           $<INSTALL_INTERFACE:include>
           $<INSTALL_INTERFACE:include/klepsydra/gen_dds_${PROJ_MAIN_NAME}>)

# Install
# ---------------------------------------------------#
install(
    TARGETS kpsr_dds_${PROJ_MAIN_NAME}_datamodel
    EXPORT KlepsydraTargets
    DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    COMPONENT libs)
install(
    DIRECTORY "include/klepsydra"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
    COMPONENT dev)
install(
    DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/gen/"
    DESTINATION
        "${CMAKE_INSTALL_INCLUDEDIR}/klepsydra/gen_dds_${PROJ_MAIN_NAME}"
    COMPONENT dev
    FILES_MATCHING
    PATTERN "*.h*")

add_dds_export_target(kpsr_dds_${PROJ_MAIN_NAME}_datamodel)

add_include_dirs(
    "${CMAKE_INSTALL_FULL_INCLUDEDIR}/klepsydra/gen_dds_${PROJ_MAIN_NAME}")

# PRINTBASICINFO(${PROJ_NAME})
