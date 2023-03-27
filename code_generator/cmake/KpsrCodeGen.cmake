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

# Klespsydra event code generator

macro(
    KpsrEventGenerator
    inputDir
    outputDir
    includePath
    disableRos
    disableZmq
    disableDds)
    find_package(Python3 REQUIRED COMPONENTS Interpreter)
    message(
        STATUS
            "Running code generator: \n${Python3_EXECUTABLE} -m kpsr_codegen -i ${inputDir} -o ${outputDir} -p ${includePath} -r ${disableRos} -z ${disableZmq} -d ${disableDds}\n"
    )
    execute_process(
        COMMAND
            ${Python3_EXECUTABLE} -m kpsr_codegen -i ${inputDir} -o ${outputDir}
            -p ${includePath} -r ${disableRos} -z ${disableZmq} -d ${disableDds}
        RESULT_VARIABLE resVar
        OUTPUT_VARIABLE outVar)
    if(resVar)
        message(
            FATAL_ERROR
                "Could not generate message files with code generator. Got output \n${outVar}"
        )
    endif()
endmacro()
