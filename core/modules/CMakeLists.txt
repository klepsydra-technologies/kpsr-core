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

# Add module code
ADD_SUBDIRECTORY(${PROJ_MAIN_NAME})
ADD_SUBDIRECTORY(mem_mdlw)
IF(KPSR_WITH_YAML)
   find_package(yaml-cpp REQUIRED)
   ADD_SUBDIRECTORY(yaml_env)
ENDIF()
IF(KPSR_WITH_DDS)
   find_package(yaml-cpp REQUIRED)
   ADD_SUBDIRECTORY(dds_mdlw)
ENDIF()
IF(KPSR_WITH_ZMQ)
   find_package(yaml-cpp REQUIRED)
   ADD_SUBDIRECTORY(zmq_mdlw)
ENDIF()
IF(KPSR_WITH_SOCKET)
   ADD_SUBDIRECTORY(socket_mdlw)
ENDIF()

add_core_export_target()
add_dds_export_target()
add_zmq_export_target()
add_include_dirs()

