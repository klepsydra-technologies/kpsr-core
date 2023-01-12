# Copyright 2023 Klepsydra Technologies AG
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


find_package(Python 3.6 REQUIRED)

find_program (BASH_PROGRAM bash REQUIRED)
if(${BASH_PROGRAM} STREQUAL "BASH_PROGRAM-NOTFOUND")
        message(FATAL_ERROR "bash not found")
endif()

function(add_nosetests)
    if (KPSR_WITH_CODE_METRICS)
        configure_file(${CMAKE_CURRENT_SOURCE_DIR}/../coveragerc.in ${CMAKE_CURRENT_BINARY_DIR}/.coveragerc @ONLY)
        add_test(
            NAME kpsr_code_gen_tests
            COMMAND ${Python_EXECUTABLE} -m coverage run $ENV{HOME}/.local/bin/nosetests -w ${CMAKE_CURRENT_SOURCE_DIR}/..
        )
        add_test(
            NAME kpsr_code_gen_coverage
            COMMAND ${Python_EXECUTABLE} -m coverage xml
        )
        set_tests_properties(kpsr_code_gen_coverage PROPERTIES DEPENDS kpsr_code_gen_cpptests)
    else()
        add_test(
            NAME kpsr_code_gen_tests
            COMMAND ${Python_EXECUTABLE} -m nose -w ${CMAKE_CURRENT_SOURCE_DIR}/..
        )
    endif()
    set_tests_properties(kpsr_code_gen_tests PROPERTIES DEPENDS kpsr_code_gen_cpptests)
endfunction()

if (Python_FOUND)
    set(PYTHON ${Python_EXECUTABLE})
    set(OUTPUT_WHEEL_FILE_NAME kpsr_codegen-1.0-py3-none-any.whl)
    set(OUTPUT_FILE ${CMAKE_BINARY_DIR}/${OUTPUT_WHEEL_FILE_NAME})
      
    file(GLOB_RECURSE ${PROJ_NAME}_PYTHON_SRCS "*.py")
    add_custom_command(
      OUTPUT ${OUTPUT_FILE}
      COMMAND ${PYTHON} -m pip install -r requirements.txt --user
      COMMAND ${PYTHON} setup.py bdist_wheel -d ${CMAKE_BINARY_DIR}
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..
      DEPENDS ${${PROJ_NAME}_PYTHON_SRCS}
    )
    add_custom_target (kpsr_code_gen_build_python
      ALL DEPENDS ${OUTPUT_FILE}
    )

    install(CODE
      "execute_process(COMMAND ${PYTHON} -m pip install --ignore-installed ${OUTPUT_FILE})"
    )
    add_nosetests()

    set(CMAKE_TEST_PREFIX_PATH ${CMAKE_PREFIX_PATH})
    LIST(APPEND CMAKE_TEST_PREFIX_PATH ${CMAKE_BINARY_DIR})
    set(CPP_TESTING_DIR     "${CMAKE_CURRENT_SOURCE_DIR}/../cpp_testing")
    set(CPP_TESTING_BASH_IN "${CMAKE_CURRENT_SOURCE_DIR}/../cpp_testing_build.sh.in")
    set(CPP_TESTING_BASH    "${CMAKE_CURRENT_BINARY_DIR}/cpp_testing_build.sh")
    configure_file(${CPP_TESTING_BASH_IN} ${CPP_TESTING_BASH} @ONLY)
    add_test (kpsr_code_gen_cpptests ${BASH_PROGRAM} ${CPP_TESTING_BASH})
    set_tests_properties(kpsr_code_gen_cpptests PROPERTIES DEPENDS kpsr_code_gen_build_python)
endif()

