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

SET(PROJ_NAME kpsr_dds_${PROJ_MAIN_NAME}_tests_1_5)
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
file(REMOVE_RECURSE ${CMAKE_CURRENT_SOURCE_DIR}/temp_idls)

file(GLOB idls
  ${CMAKE_SOURCE_DIR}/modules/codegen/gen_test1/dds/idl/quaternion_data.idl
  ${CMAKE_SOURCE_DIR}/modules/codegen/gen_test1/dds/idl/gps_data.idl
  ${CMAKE_SOURCE_DIR}/modules/codegen/gen_test5/dds/idl/*.idl)

file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/temp_idls)
file(COPY ${idls} DESTINATION  ${CMAKE_CURRENT_SOURCE_DIR}/temp_idls)

file(GLOB temp_idls ${CMAKE_CURRENT_SOURCE_DIR}/temp_idls/*.idl)
FOREACH(idl ${temp_idls})
        Connext_IDLGEN (${idl} ${CMAKE_CURRENT_SOURCE_DIR})
        GET_FILENAME_COMPONENT(nfile ${idl} NAME)
        Connext_GENDATAMODEL(${nfile} ${CMAKE_CURRENT_SOURCE_DIR})
ENDFOREACH(idl)

# Include directories with headers
#---------------------------------------------------#
INCLUDE_DIRECTORIES( ${CMAKE_SOURCE_DIR}/modules/${PROJ_MAIN_NAME}/include )
INCLUDE_DIRECTORIES( ${Connext_INCLUDE_DIRS} )
INCLUDE_DIRECTORIES( ${CMAKE_SOURCE_DIR}/modules/codegen/gen_test1/poco/include )
INCLUDE_DIRECTORIES( ${CMAKE_SOURCE_DIR}/modules/codegen/gen_test1/dds/include )
INCLUDE_DIRECTORIES( ${CMAKE_SOURCE_DIR}/modules/codegen/gen_test5/poco/include )
INCLUDE_DIRECTORIES( ${CMAKE_SOURCE_DIR}/modules/codegen/gen_test5/dds/include )
INCLUDE_DIRECTORIES( ${CMAKE_CURRENT_SOURCE_DIR}/gen )
INCLUDE_DIRECTORIES( ${CMAKE_SOURCE_DIR}/../../core/modules/core/include )
INCLUDE_DIRECTORIES( ${CMAKE_SOURCE_DIR}/../../core/modules/dds_mdlw/include )
INCLUDE_DIRECTORIES( ${CMAKE_SOURCE_DIR}/../../serialization/modules/serialization/include )
INCLUDE_DIRECTORIES( ${CMAKE_SOURCE_DIR}/../../serialization/modules/dds_mdlw/include )
INCLUDE_DIRECTORIES( ${CMAKE_SOURCE_DIR}/../../serialization/modules/dds_mdlw/gen)
INCLUDE_DIRECTORIES( ${THIRDPARTIES_PATH}/include )

# Source
#---------------------------------------------------#
FILE(GLOB ${PROJ_NAME}_SRC
    "src/*.cpp"
)

# Link libraries with Project
#---------------------------------------------------#
LINK_DIRECTORIES( ${CMAKE_BINARY_DIR}/../../../bin/ )

ADD_LIBRARY (kpsr_dds_${PROJ_MAIN_NAME}_datamodel_1_5 SHARED ${Connext_DATAMODEL})

ADD_EXECUTABLE(${PROJ_NAME} ${${PROJ_NAME}_HEADERS} ${${PROJ_NAME}_SRC} )

#---------------------------------------------------#
TARGET_LINK_LIBRARIES(kpsr_dds_${PROJ_MAIN_NAME}_datamodel_1_5 atomic ${Connext_LIBRARIES})

TARGET_LINK_LIBRARIES(${PROJ_NAME} kpsr_core gtest_main
    kpsr_dds_serialization_datamodel kpsr_dds_${PROJ_MAIN_NAME}_datamodel_1_5 atomic ${Connext_LIBRARIES})

add_test(NAME ${PROJ_NAME} COMMAND ${PROJ_NAME} --output-on-failure --gtest_output=xml:gtestresults.xml)
