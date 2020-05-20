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

SET(PROJ_NAME kpsr_zmq_${PROJ_MAIN_NAME}_tests)
PROJECT(${PROJ_NAME})

#find cppzmq wrapper, installed by make of cppzmq
find_package(cppzmq)
# Include directories with headers
#---------------------------------------------------#
INCLUDE_DIRECTORIES( ${CMAKE_SOURCE_DIR}/modules/${PROJ_MAIN_NAME}/include )
include_directories( ${cppzmq_INCLUDE_DIR})
INCLUDE_DIRECTORIES( ${CMAKE_SOURCE_DIR}/modules/codegen/gen_test11/poco/include )
INCLUDE_DIRECTORIES( ${CMAKE_SOURCE_DIR}/modules/codegen/gen_test11/serialization/include )
INCLUDE_DIRECTORIES( ${CMAKE_SOURCE_DIR}/modules/codegen/gen_test21/poco/include )
INCLUDE_DIRECTORIES( ${CMAKE_SOURCE_DIR}/modules/codegen/gen_test21/serialization/include )
INCLUDE_DIRECTORIES( ${CMAKE_SOURCE_DIR}/modules/codegen/gen_test31/poco/include )
INCLUDE_DIRECTORIES( ${CMAKE_SOURCE_DIR}/modules/codegen/gen_test31/serialization/include )
INCLUDE_DIRECTORIES( ${CMAKE_SOURCE_DIR}/modules/codegen/gen_test41/poco/include )
INCLUDE_DIRECTORIES( ${CMAKE_SOURCE_DIR}/modules/codegen/gen_test41/serialization/include )
INCLUDE_DIRECTORIES( ${CMAKE_SOURCE_DIR}/modules/codegen/gen_test51/poco/include )
INCLUDE_DIRECTORIES( ${CMAKE_SOURCE_DIR}/modules/codegen/gen_test51/serialization/include )
INCLUDE_DIRECTORIES( ${CMAKE_SOURCE_DIR}/../../core/modules/core/include )
INCLUDE_DIRECTORIES( ${CMAKE_SOURCE_DIR}/../../core/modules/zmq_mdlw/include )
INCLUDE_DIRECTORIES( ${CMAKE_SOURCE_DIR}/../../serialization/modules/serialization/include )
INCLUDE_DIRECTORIES( ${THIRDPARTIES_PATH}/include )

# Source
#---------------------------------------------------#
FILE(GLOB ${PROJ_NAME}_SRC
    "src/*.cpp"
)

# Link libraries with Project
#---------------------------------------------------#
LINK_DIRECTORIES( ${CMAKE_BINARY_DIR}/../../../bin/ )

ADD_EXECUTABLE(${PROJ_NAME} ${${PROJ_NAME}_HEADERS} ${${PROJ_NAME}_SRC} )

#---------------------------------------------------#
TARGET_LINK_LIBRARIES(${PROJ_NAME} kpsr_core cppzmq gtest_main atomic )

add_test(NAME ${PROJ_NAME} COMMAND ${PROJ_NAME} --output-on-failure --gtest_output=xml:gtestresults.xml)
