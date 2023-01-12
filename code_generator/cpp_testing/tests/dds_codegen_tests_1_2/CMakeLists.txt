# Copyright 2023 Klepsydra Technologies AG
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


SET(PROJ_NAME kpsr_dds_${PROJ_MAIN_NAME}_tests_1_2)
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

file(GLOB idls ${CMAKE_SOURCE_DIR}/modules/codegen/gen_test1/dds/idl/*.idl
               ${CMAKE_SOURCE_DIR}/modules/codegen/gen_test2/dds/idl/*.idl)
FOREACH(idl ${idls})
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
INCLUDE_DIRECTORIES( ${CMAKE_SOURCE_DIR}/modules/codegen/gen_test2/poco/include )
INCLUDE_DIRECTORIES( ${CMAKE_SOURCE_DIR}/modules/codegen/gen_test2/dds/include )
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

ADD_LIBRARY (kpsr_dds_${PROJ_MAIN_NAME}_datamodel_1_2 SHARED ${Connext_DATAMODEL})

ADD_EXECUTABLE(${PROJ_NAME} ${${PROJ_NAME}_HEADERS} ${${PROJ_NAME}_SRC} )

# Link libraries with Project
#---------------------------------------------------#
TARGET_LINK_LIBRARIES(kpsr_dds_${PROJ_MAIN_NAME}_datamodel_1_2 atomic ${Connext_LIBRARIES})

TARGET_LINK_LIBRARIES(${PROJ_NAME} kpsr_core gtest_main
    kpsr_dds_serialization_datamodel kpsr_dds_${PROJ_MAIN_NAME}_datamodel_1_2 atomic ${Connext_LIBRARIES})

add_test(NAME ${PROJ_NAME} COMMAND ${PROJ_NAME} --output-on-failure --gtest_output=xml:gtestresults.xml)
