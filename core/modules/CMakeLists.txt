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

