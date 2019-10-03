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

SET(PROJ_NAME kpsr_socket_${PROJ_MAIN_NAME})
PROJECT(${PROJ_NAME})

# Include directories with headers
#---------------------------------------------------#
INCLUDE_DIRECTORIES( ${CMAKE_SOURCE_DIR}/${PROJ_MAIN_NAME}/modules/${PROJ_MAIN_NAME}/include )
INCLUDE_DIRECTORIES( ${CMAKE_CURRENT_SOURCE_DIR}/include )
INCLUDE_DIRECTORIES( ${THIRDPARTIES_PATH}/include )

# Source
#---------------------------------------------------#
FILE(GLOB ${PROJ_NAME}_SRC
    "src/*.cpp"
)

FILE(GLOB_RECURSE ${PROJ_NAME}_HEADERS
    "include/*"
)

# Create Library
#---------------------------------------------------#
LINK_DIRECTORIES()

ADD_LIBRARY(${PROJ_NAME} ${${PROJ_MAIN_NAME}_LIB_TYPE} ${${PROJ_NAME}_HEADERS} ${${PROJ_NAME}_SRC} )

# Link libraries with Project
#---------------------------------------------------#
TARGET_LINK_LIBRARIES(${PROJ_NAME}  ${CMAKE_THREAD_LIBS_INIT})

if (KPSR_WITH_CODE_METRICS)
    add_cppcheck(${PROJ_NAME} UNUSED_FUNCTIONS STYLE POSSIBLE_ERROR)
endif()

# Install
#---------------------------------------------------#
INSTALL(TARGETS ${PROJ_NAME}
    DESTINATION "${CMAKE_INSTALL_LIBDIR}"
)
INSTALL(DIRECTORY "include/klepsydra"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
)

add_core_export_target(${PROJ_NAME})

#PRINTBASICINFO(${PROJ_NAME})

