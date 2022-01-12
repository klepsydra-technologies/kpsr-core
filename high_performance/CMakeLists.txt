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

set(PROJ_MAIN_NAME "high_performance")
project(${PROJ_MAIN_NAME})

# ##############################################################################
# ADD SUB FOLDERS
# ##############################################################################
add_subdirectory(modules)
if(KPSR_ENABLE_TESTS)
    add_subdirectory(tests)
endif()

list(APPEND COVERAGE_EXCLUDES ${CMAKE_CURRENT_SOURCE_DIR}/tests
     ${CMAKE_CURRENT_SOURCE_DIR}/examples)
set(COVERAGE_EXCLUDES
    ${COVERAGE_EXCLUDES}
    PARENT_SCOPE)
add_core_export_target()
add_include_dirs()
