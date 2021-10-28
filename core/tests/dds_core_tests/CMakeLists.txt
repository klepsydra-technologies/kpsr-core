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

SET(PROJ_NAME kpsr_dds_${PROJ_MAIN_NAME}_tests)
PROJECT(${PROJ_NAME})

include (MacroConnext)
find_package (Connext REQUIRED)
SET (DEFINITIONS ${DEFINITIONS} " -DDDS_IMPLEMENTATION_${DDS_IMPLEMENTATION}")

add_definitions (
  ${Connext_DEFINITIONS}
  ${DEFINITIONS}
)

# Source
#---------------------------------------------------#
FILE(GLOB ${PROJ_NAME}_SRC
    "src/*.cpp"
)

# Link libraries with Project
#---------------------------------------------------#
ADD_EXECUTABLE(${PROJ_NAME} ${${PROJ_NAME}_HEADERS} ${${PROJ_NAME}_SRC} )

#---------------------------------------------------#
TARGET_LINK_LIBRARIES(${PROJ_NAME} kpsr_core kpsr_core_yaml yaml-cpp gtest_main kpsr_dds_${PROJ_MAIN_NAME}
  kpsr_dds_core kpsr_dds_serialization_datamodel kpsr_dds_${PROJ_MAIN_NAME}_datamodel ${Connext_LIBRARIES} kpsr_serialization)
TARGET_INCLUDE_DIRECTORIES(${PROJ_NAME}
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_SOURCE_DIR}/${PROJ_MAIN_NAME}/tests/common
  )

add_test(NAME ${PROJ_NAME} COMMAND ${PROJ_NAME} --output-on-failure --gtest_output=xml:gtestresults.xml)
