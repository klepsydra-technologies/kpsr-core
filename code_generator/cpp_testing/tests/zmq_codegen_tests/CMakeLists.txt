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

# Source
#---------------------------------------------------#
FILE(GLOB ${PROJ_NAME}_SRC
    "src/*.cpp"
)

ADD_EXECUTABLE(${PROJ_NAME} ${${PROJ_NAME}_HEADERS} ${${PROJ_NAME}_SRC} )

# Link libraries with Project
#---------------------------------------------------#
TARGET_LINK_LIBRARIES(${PROJ_NAME} PUBLIC kpsr_core kpsr_zmq_core gtest_main )

TARGET_INCLUDE_DIRECTORIES(${PROJ_NAME}
  PRIVATE
  ${CMAKE_SOURCE_DIR}/modules/${PROJ_MAIN_NAME}/include
  ${CMAKE_SOURCE_DIR}/modules/codegen/gen_test11/poco/include
  ${CMAKE_SOURCE_DIR}/modules/codegen/gen_test11/serialization/include
  ${CMAKE_SOURCE_DIR}/modules/codegen/gen_test21/poco/include
  ${CMAKE_SOURCE_DIR}/modules/codegen/gen_test21/serialization/include
  ${CMAKE_SOURCE_DIR}/modules/codegen/gen_test31/poco/include
  ${CMAKE_SOURCE_DIR}/modules/codegen/gen_test31/serialization/include
  ${CMAKE_SOURCE_DIR}/modules/codegen/gen_test41/poco/include
  ${CMAKE_SOURCE_DIR}/modules/codegen/gen_test41/serialization/include
  ${CMAKE_SOURCE_DIR}/modules/codegen/gen_test51/poco/include
  ${CMAKE_SOURCE_DIR}/modules/codegen/gen_test51/serialization/include
  )

add_test(NAME ${PROJ_NAME} COMMAND ${PROJ_NAME} --output-on-failure --gtest_output=xml:gtestresults.xml)
