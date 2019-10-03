#****************************************************************************
#
#                           Klepsydra Core Modules
#              Copyright (C) 2019-2020  Klepsydra Technologies GmbH
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
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
INCLUDE_DIRECTORIES( ${${PROJ_MAIN_NAME}_PATH_MAIN}/modules/${PROJ_MAIN_NAME}/include )
INCLUDE_DIRECTORIES( ${Connext_INCLUDE_DIRS} )
INCLUDE_DIRECTORIES( ${CMAKE_CURRENT_SOURCE_DIR}/gen )
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
LINK_DIRECTORIES( ${${PROJ_MAIN_NAME}_PATH_LIBS} )
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

