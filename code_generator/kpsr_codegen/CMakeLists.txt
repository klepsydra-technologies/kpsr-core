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

find_program(PYTHON "python3")
if(${PYTHON} STREQUAL "PYTHON-NOTFOUND")
	message(FATAL_ERROR "Python3 not found")
endif()

find_program (BASH_PROGRAM bash REQUIRED)
if(${BASH_PROGRAM} STREQUAL "BASH_PROGRAM-NOTFOUND")
	message(FATAL_ERROR "bash not found")
endif()

function(add_nosetests)
    if (KPSR_WITH_CODE_METRICS)
        configure_file(${CMAKE_CURRENT_SOURCE_DIR}/../coveragerc.in ${CMAKE_CURRENT_BINARY_DIR}/.coveragerc @ONLY)
        add_test(
            NAME kpsr_code_gen_tests
            COMMAND ${PYTHON} -m coverage run $ENV{HOME}/.local/bin/nosetests -w ${CMAKE_CURRENT_SOURCE_DIR}/..
        )
        add_test(
            NAME kpsr_code_gen_coverage
            COMMAND ${PYTHON} -m coverage xml
        )
    else()
        add_test(
            NAME kpsr_code_gen_tests
            COMMAND ${PYTHON} -m nose -w ${CMAKE_CURRENT_SOURCE_DIR}/..
        )
    endif()
endfunction()

if (PYTHON)
    set(INSTALL_PYTHON_BASH_IN "${CMAKE_CURRENT_SOURCE_DIR}/../install_python.sh.in")
    set(INSTALL_PYTHON_BASH    "${CMAKE_CURRENT_BINARY_DIR}/install_python.sh")
    configure_file(${INSTALL_PYTHON_BASH_IN} ${INSTALL_PYTHON_BASH} @ONLY)
    add_test (NAME kpsr_code_gen_install_python
        COMMAND ${BASH_PROGRAM} ${INSTALL_PYTHON_BASH}
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..
    )

    add_nosetests()

    set(CPP_TESTING_DIR     "${CMAKE_CURRENT_SOURCE_DIR}/../cpp_testing")
    set(CPP_TESTING_BASH_IN "${CMAKE_CURRENT_SOURCE_DIR}/../cpp_testing_build.sh.in")
    set(CPP_TESTING_BASH    "${CMAKE_CURRENT_BINARY_DIR}/cpp_testing_build.sh")
    configure_file(${CPP_TESTING_BASH_IN} ${CPP_TESTING_BASH} @ONLY)
    add_test (kpsr_code_gen_cpptests ${BASH_PROGRAM} ${CPP_TESTING_BASH})
endif()

