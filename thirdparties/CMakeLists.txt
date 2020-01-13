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
