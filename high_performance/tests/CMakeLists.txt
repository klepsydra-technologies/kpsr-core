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

# Prevent overriding the parent project's compiler/linker
# settings on Windows
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

# Add googletest directly to our build. This defines
# the gtest and gtest_main targets.
if(NOT TARGET gtest)
   add_subdirectory(${GTEST_PATH}
                    ${CMAKE_BINARY_DIR}/googletest-build
                    EXCLUDE_FROM_ALL)
endif()

# The gtest/gtest_main targets carry header search path
# dependencies automatically when using CMake 2.8.11 or
# later. Otherwise we have to add them here ourselves.
include_directories("${GTEST_PATH}/googletest/include")
include_directories("${GTEST_PATH}/googlemock/include")

add_subdirectory(${PROJ_MAIN_NAME}_tests)
add_subdirectory(disruptor4cpp_test)
IF(KPSR_WITH_ZMQ)
   ADD_SUBDIRECTORY(zmq_${PROJ_MAIN_NAME}_tests)
ENDIF()
IF(KPSR_WITH_DDS)
   ADD_SUBDIRECTORY(dds_${PROJ_MAIN_NAME}_tests)
ENDIF()
IF(KPSR_TEST_PERFORMANCE)
   ADD_SUBDIRECTORY(performance)
ENDIF()
