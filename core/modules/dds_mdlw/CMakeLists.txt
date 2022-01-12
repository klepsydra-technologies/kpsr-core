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

include(MacroConnext)
find_package(Connext REQUIRED)
set(DEFINITIONS ${DEFINITIONS} " -DDDS_IMPLEMENTATION_${DDS_IMPLEMENTATION}")

add_definitions(${Connext_DEFINITIONS} ${DEFINITIONS})

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
file(GLOB ${PROJ_NAME}_SRC "src/*.cpp")
file(GLOB_RECURSE ${PROJ_NAME}_HEADERS "include/*.h")
file(GLOB ${PROJ_NAME}_GEN_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/gen/*.h*")

# Create Library
# ---------------------------------------------------#
add_library(${PROJ_NAME} ${${PROJ_MAIN_NAME}_LIB_TYPE} ${${PROJ_NAME}_HEADERS}
                         ${${PROJ_NAME}_SRC})

add_library(kpsr_dds_${PROJ_MAIN_NAME}_datamodel SHARED ${Connext_DATAMODEL})

# Link libraries with Project
# ---------------------------------------------------#
target_link_libraries(
    ${PROJ_NAME} kpsr_${PROJ_MAIN_NAME} kpsr_dds_${PROJ_MAIN_NAME}_datamodel
    ${Connext_LIBRARIES} kpsr_${PROJ_MAIN_NAME}_yaml)
target_link_libraries(kpsr_dds_${PROJ_MAIN_NAME}_datamodel
                      PUBLIC ${Connext_LIBRARIES})

target_include_directories(
    ${PROJ_NAME}
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/gen>
           $<INSTALL_INTERFACE:include>
           $<INSTALL_INTERFACE:include/klepsydra/gen_dds_${PROJ_MAIN_NAME}>)
target_include_directories(kpsr_dds_${PROJ_MAIN_NAME}_datamodel
                           PUBLIC ${Connext_INCLUDE_DIRS})
if(KPSR_WITH_CODE_METRICS)
    add_cppcheck(${PROJ_NAME} UNUSED_FUNCTIONS STYLE POSSIBLE_ERROR)
endif()

set_target_properties(
    ${PROJ_NAME} PROPERTIES VERSION ${KLEPSYDRA_CORE_VERSION}
                            SOVERSION ${KLEPSYDRA_CORE_VERSION_MAJOR})

set_target_properties(
    kpsr_dds_${PROJ_MAIN_NAME}_datamodel
    PROPERTIES VERSION ${KLEPSYDRA_CORE_VERSION}
               SOVERSION ${KLEPSYDRA_CORE_VERSION_MAJOR})

# Install
# ---------------------------------------------------#
install(
    TARGETS ${PROJ_NAME}
    EXPORT KlepsydraTargets
    DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    COMPONENT libs)
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
    FILES_MATCHING
    PATTERN "*.h*")

add_dds_export_target(${PROJ_NAME})
add_dds_export_target(kpsr_dds_${PROJ_MAIN_NAME}_datamodel)

add_include_dirs(
    "${CMAKE_INSTALL_FULL_INCLUDEDIR}/klepsydra/gen_dds_${PROJ_MAIN_NAME}")

# PRINTBASICINFO(${PROJ_NAME})
