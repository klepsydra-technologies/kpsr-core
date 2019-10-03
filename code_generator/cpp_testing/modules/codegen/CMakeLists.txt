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

SET(PROJ_NAME kpsr_${PROJ_MAIN_NAME})
PROJECT(${PROJ_NAME})

KpsrEventGenerator("${CMAKE_CURRENT_SOURCE_DIR}/kidl_test1" "${CMAKE_CURRENT_SOURCE_DIR}/gen_test1" "klepsydra/codegen" "False" "True" "False")
KpsrEventGenerator("${CMAKE_CURRENT_SOURCE_DIR}/kidl_test2" "${CMAKE_CURRENT_SOURCE_DIR}/gen_test2" "klepsydra/codegen" "False" "True" "False")
KpsrEventGenerator("${CMAKE_CURRENT_SOURCE_DIR}/kidl_test3" "${CMAKE_CURRENT_SOURCE_DIR}/gen_test3" "klepsydra/codegen" "False" "True" "False")
KpsrEventGenerator("${CMAKE_CURRENT_SOURCE_DIR}/kidl_test4" "${CMAKE_CURRENT_SOURCE_DIR}/gen_test4" "klepsydra/codegen" "False" "True" "False")

KpsrEventGenerator("${CMAKE_CURRENT_SOURCE_DIR}/kidl_test1" "${CMAKE_CURRENT_SOURCE_DIR}/gen_test11" "klepsydra/codegen" "True" "False" "True")
KpsrEventGenerator("${CMAKE_CURRENT_SOURCE_DIR}/kidl_test2" "${CMAKE_CURRENT_SOURCE_DIR}/gen_test21" "klepsydra/codegen" "True" "False" "True")
KpsrEventGenerator("${CMAKE_CURRENT_SOURCE_DIR}/kidl_test3" "${CMAKE_CURRENT_SOURCE_DIR}/gen_test31" "klepsydra/codegen" "True" "False" "True")
KpsrEventGenerator("${CMAKE_CURRENT_SOURCE_DIR}/kidl_test4" "${CMAKE_CURRENT_SOURCE_DIR}/gen_test41" "klepsydra/codegen" "True" "False" "True")

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

file(GLOB TEST2_ROS_MESSAGES
  "gen_test2/rosstg/msg/*.msg"
)

file(GLOB TEST3_ROS_MESSAGES
  "gen_test3/rosstg/msg/*.msg"
)

file(GLOB TEST4_ROS_MESSAGES
  "gen_test4/rosstg/msg/*.msg"
)

file(COPY ${TEST1_ROS_MAPPERS} DESTINATION ${CMAKE_SOURCE_DIR}/tests/kpsr_ros_codegen/include/kpsr_ros_codegen)
file(COPY ${TEST2_ROS_MAPPERS} DESTINATION ${CMAKE_SOURCE_DIR}/tests/kpsr_ros_codegen/include/kpsr_ros_codegen)
file(COPY ${TEST3_ROS_MAPPERS} DESTINATION ${CMAKE_SOURCE_DIR}/tests/kpsr_ros_codegen/include/kpsr_ros_codegen)
file(COPY ${TEST4_ROS_MAPPERS} DESTINATION ${CMAKE_SOURCE_DIR}/tests/kpsr_ros_codegen/include/kpsr_ros_codegen)
file(COPY ${TEST2_ROS_MESSAGES} DESTINATION ${CMAKE_SOURCE_DIR}/tests/kpsr_ros_codegen/msg)
file(COPY ${TEST3_ROS_MESSAGES} DESTINATION ${CMAKE_SOURCE_DIR}/tests/kpsr_ros_codegen/msg)
file(COPY ${TEST4_ROS_MESSAGES} DESTINATION ${CMAKE_SOURCE_DIR}/tests/kpsr_ros_codegen/msg)

