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

SET(PROJ_NAME kpsr_${PROJ_MAIN_NAME})
PROJECT(${PROJ_NAME})

KpsrEventGenerator("${CMAKE_CURRENT_SOURCE_DIR}/kidl_test1" "${CMAKE_CURRENT_SOURCE_DIR}/gen_test1" "klepsydra/codegen" "False" "True" "False")
KpsrEventGenerator("${CMAKE_CURRENT_SOURCE_DIR}/kidl_test2" "${CMAKE_CURRENT_SOURCE_DIR}/gen_test2" "klepsydra/codegen" "False" "True" "False")
KpsrEventGenerator("${CMAKE_CURRENT_SOURCE_DIR}/kidl_test3" "${CMAKE_CURRENT_SOURCE_DIR}/gen_test3" "klepsydra/codegen" "False" "True" "False")
KpsrEventGenerator("${CMAKE_CURRENT_SOURCE_DIR}/kidl_test4" "${CMAKE_CURRENT_SOURCE_DIR}/gen_test4" "klepsydra/codegen" "False" "True" "False")
KpsrEventGenerator("${CMAKE_CURRENT_SOURCE_DIR}/kidl_test5" "${CMAKE_CURRENT_SOURCE_DIR}/gen_test5" "klepsydra/codegen" "False" "True" "False")

KpsrEventGenerator("${CMAKE_CURRENT_SOURCE_DIR}/kidl_test1" "${CMAKE_CURRENT_SOURCE_DIR}/gen_test11" "klepsydra/codegen" "True" "False" "True")
KpsrEventGenerator("${CMAKE_CURRENT_SOURCE_DIR}/kidl_test2" "${CMAKE_CURRENT_SOURCE_DIR}/gen_test21" "klepsydra/codegen" "True" "False" "True")
KpsrEventGenerator("${CMAKE_CURRENT_SOURCE_DIR}/kidl_test3" "${CMAKE_CURRENT_SOURCE_DIR}/gen_test31" "klepsydra/codegen" "True" "False" "True")
KpsrEventGenerator("${CMAKE_CURRENT_SOURCE_DIR}/kidl_test4" "${CMAKE_CURRENT_SOURCE_DIR}/gen_test41" "klepsydra/codegen" "True" "False" "True")
KpsrEventGenerator("${CMAKE_CURRENT_SOURCE_DIR}/kidl_test5" "${CMAKE_CURRENT_SOURCE_DIR}/gen_test51" "klepsydra/codegen" "True" "False" "True")

# Source
#---------------------------------------------------#
FILE(GLOB_RECURSE ${PROJ_NAME}_HEADERS
    "gen_test1/include/*.h"
)

# Source
#---------------------------------------------------#
FILE(GLOB_RECURSE ${PROJ_NAME}_HEADERS
    "gen_test2/include/*.h"
)

file(GLOB_RECURSE TEST1_ROS_MAPPERS
  "gen_test1/rosstg/include/*.h"
)

file(GLOB_RECURSE TEST2_ROS_MAPPERS
  "gen_test2/rosstg/include/*.h"
)

file(GLOB_RECURSE TEST3_ROS_MAPPERS
  "gen_test3/rosstg/include/*.h"
)

file(GLOB_RECURSE TEST4_ROS_MAPPERS
  "gen_test4/rosstg/include/*.h"
)

file(GLOB_RECURSE TEST5_ROS_MAPPERS
  "gen_test5/rosstg/include/*.h"
)

file(GLOB TEST2_ROS_MESSAGES
  "gen_test2/rosstg/msg/*.msg"
)

file(GLOB TEST3_ROS_MESSAGES
  "gen_test3/rosstg/msg/*.msg"
)

file(GLOB TEST4_ROS_MESSAGES
  "gen_test4/rosstg/msg/*.msg"
)

file(GLOB TEST5_ROS_MESSAGES
  "gen_test5/rosstg/msg/*.msg"
)

file(COPY ${TEST1_ROS_MAPPERS} DESTINATION ${CMAKE_SOURCE_DIR}/tests/kpsr_ros_codegen/include/kpsr_ros_codegen)
file(COPY ${TEST2_ROS_MAPPERS} DESTINATION ${CMAKE_SOURCE_DIR}/tests/kpsr_ros_codegen/include/kpsr_ros_codegen)
file(COPY ${TEST3_ROS_MAPPERS} DESTINATION ${CMAKE_SOURCE_DIR}/tests/kpsr_ros_codegen/include/kpsr_ros_codegen)
file(COPY ${TEST4_ROS_MAPPERS} DESTINATION ${CMAKE_SOURCE_DIR}/tests/kpsr_ros_codegen/include/kpsr_ros_codegen)
file(COPY ${TEST5_ROS_MAPPERS} DESTINATION ${CMAKE_SOURCE_DIR}/tests/kpsr_ros_codegen/include/kpsr_ros_codegen)
file(COPY ${TEST2_ROS_MESSAGES} DESTINATION ${CMAKE_SOURCE_DIR}/tests/kpsr_ros_codegen/msg)
file(COPY ${TEST3_ROS_MESSAGES} DESTINATION ${CMAKE_SOURCE_DIR}/tests/kpsr_ros_codegen/msg)
file(COPY ${TEST4_ROS_MESSAGES} DESTINATION ${CMAKE_SOURCE_DIR}/tests/kpsr_ros_codegen/msg)
file(COPY ${TEST5_ROS_MESSAGES} DESTINATION ${CMAKE_SOURCE_DIR}/tests/kpsr_ros_codegen/msg)

