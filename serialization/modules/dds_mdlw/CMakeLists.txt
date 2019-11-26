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

SET(PROJ_NAME kpsr_dds_${PROJ_MAIN_NAME})
PROJECT(${PROJ_NAME})

include (MacroConnext)
find_package (Connext REQUIRED)
SET (DEFINITIONS ${DEFINITIONS} " -DDDS_IMPLEMENTATION_${DDS_IMPLEMENTATION}")

add_definitions (
  ${Connext_DEFINITIONS}
  ${DEFINITIONS}
)

file(REMOVE_RECURSE ${CMAKE_CURRENT_SOURCE_DIR}/gen)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/gen)

file(GLOB idls "./idl/*.idl")
FOREACH(idl ${idls})
        Connext_IDLGEN (${idl} ${CMAKE_CURRENT_SOURCE_DIR})
        GET_FILENAME_COMPONENT(nfile ${idl} NAME)
        Connext_GENDATAMODEL(${nfile} ${CMAKE_CURRENT_SOURCE_DIR})
ENDFOREACH(idl)

# Include directories with headers
#---------------------------------------------------#
INCLUDE_DIRECTORIES( ${CMAKE_CURRENT_SOURCE_DIR}/include )
INCLUDE_DIRECTORIES( ${CMAKE_CURRENT_SOURCE_DIR}/gen )
INCLUDE_DIRECTORIES( ${CMAKE_SOURCE_DIR}/${PROJ_MAIN_NAME}/modules/${PROJ_MAIN_NAME}/include )
INCLUDE_DIRECTORIES( ${Connext_INCLUDE_DIRS} )
INCLUDE_DIRECTORIES( ${THIRDPARTIES_PATH}/include )

# Source
#---------------------------------------------------#
FILE(GLOB_RECURSE ${PROJ_NAME}_HEADERS
    "include/*.h"
)
FILE(GLOB ${PROJ_NAME}_GEN_HEADERS
    "${CMAKE_CURRENT_SOURCE_DIR}/gen/*.h*"
)

# Create Library
#---------------------------------------------------#
LINK_DIRECTORIES()
# ADD_LIBRARY(${PROJ_NAME} ${${PROJ_MAIN_NAME}_LIB_TYPE} ${${PROJ_NAME}_HEADERS} ${${PROJ_NAME}_SRC} )

ADD_LIBRARY (kpsr_dds_${PROJ_MAIN_NAME}_datamodel SHARED ${Connext_DATAMODEL})

# Link libraries with Project
#---------------------------------------------------#
# TARGET_LINK_LIBRARIES(${PROJ_NAME})
TARGET_LINK_LIBRARIES(kpsr_dds_${PROJ_MAIN_NAME}_datamodel ${Connext_LIBRARIES})

# Install
#---------------------------------------------------#
INSTALL(TARGETS kpsr_dds_${PROJ_MAIN_NAME}_datamodel
    DESTINATION "${CMAKE_INSTALL_LIBDIR}"
)
INSTALL(DIRECTORY "include/klepsydra"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
)
INSTALL(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/gen/"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/klepsydra/gen_dds_${PROJ_MAIN_NAME}"
    FILES_MATCHING PATTERN "*.h*"
)

add_dds_export_target(kpsr_dds_${PROJ_MAIN_NAME}_datamodel)

add_include_dirs("${CMAKE_INSTALL_FULL_INCLUDEDIR}/klepsydra/gen_dds_${PROJ_MAIN_NAME}")

#PRINTBASICINFO(${PROJ_NAME})

