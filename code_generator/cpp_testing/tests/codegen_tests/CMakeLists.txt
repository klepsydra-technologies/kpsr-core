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

SET(PROJ_NAME kpsr_${PROJ_MAIN_NAME}_tests)
PROJECT(${PROJ_NAME})

# Include directories with headers
#---------------------------------------------------#
INCLUDE_DIRECTORIES( ${CMAKE_SOURCE_DIR}/modules/${PROJ_MAIN_NAME}/gen_test1/poco/include )
INCLUDE_DIRECTORIES( ${CMAKE_SOURCE_DIR}/modules/${PROJ_MAIN_NAME}/gen_test2/poco/include )
INCLUDE_DIRECTORIES( ${CMAKE_SOURCE_DIR}/modules/${PROJ_MAIN_NAME}/gen_test3/poco/include )
INCLUDE_DIRECTORIES( ${CMAKE_SOURCE_DIR}/modules/${PROJ_MAIN_NAME}/gen_test4/poco/include )
INCLUDE_DIRECTORIES( ${CMAKE_SOURCE_DIR}/modules/${PROJ_MAIN_NAME}/include )
INCLUDE_DIRECTORIES( ${CMAKE_SOURCE_DIR}/../../core/modules/core/include )
INCLUDE_DIRECTORIES( ${THIRDPARTIES_PATH}/include )

# Source
#---------------------------------------------------#
FILE(GLOB ${PROJ_NAME}_SRC
    "src/*.cpp"
)

# Link libraries with Project
#---------------------------------------------------#
LINK_DIRECTORIES( ${CMAKE_BINARY_DIR}/../../../core/modules/core/ )

# Create Library
#---------------------------------------------------#
ADD_EXECUTABLE(${PROJ_NAME} ${${PROJ_NAME}_HEADERS} ${${PROJ_NAME}_SRC} )

TARGET_LINK_LIBRARIES(${PROJ_NAME} kpsr_core gtest_main atomic )

add_test(NAME ${PROJ_NAME} COMMAND ${PROJ_NAME} --output-on-failure --gtest_output=xml:gtestresults.xml)
