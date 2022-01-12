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
add_subdirectory(${PROJ_MAIN_NAME})
add_subdirectory(mem_mdlw)
if(KPSR_WITH_YAML)
    add_subdirectory(yaml_env)
endif()
if(KPSR_WITH_DDS)
    add_subdirectory(dds_mdlw)
endif()
if(KPSR_WITH_ZMQ)
    add_subdirectory(zmq_mdlw)
endif()
if(KPSR_WITH_SOCKET)
    add_subdirectory(socket_mdlw)
endif()

add_core_export_target()
add_dds_export_target()
add_zmq_export_target()
add_include_dirs()
