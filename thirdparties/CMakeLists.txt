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

SET(PROJ_MAIN_NAME "thirdparties")
PROJECT(${PROJ_MAIN_NAME})

file(COPY googletest DESTINATION ${THIRDPARTIES_PATH})
file(COPY cereal/include DESTINATION ${THIRDPARTIES_PATH})
file(COPY spdlog/include DESTINATION ${THIRDPARTIES_PATH})

file(GLOB CONCURRENTQUEUE_HEADERS
  "concurrentqueue/*.h"
)
file(COPY ${CONCURRENTQUEUE_HEADERS} DESTINATION ${THIRDPARTIES_PATH}/include)

# Install
#---------------------------------------------------#
INSTALL(DIRECTORY "${THIRDPARTIES_PATH}/include"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/kpsr_3parties"
)

add_include_dirs("${CMAKE_INSTALL_FULL_INCLUDEDIR}/kpsr_3parties/include")
