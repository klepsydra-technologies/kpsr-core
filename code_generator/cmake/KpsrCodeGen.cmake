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

macro(KpsrEventGenerator inputDir outputDir includePath disableRos disableZmq disableDds)
   MESSAGE("python3 -m kpsr_codegen -i ${inputDir} -o ${outputDir} -p ${includePath} -r ${disableRos} -z ${disableZmq} -d ${disableDds}")
   execute_process (
      COMMAND bash -c "python3 -m kpsr_codegen -i ${inputDir} -o ${outputDir} -p ${includePath} -r ${disableRos} -z ${disableZmq} -d ${disableDds}"
      OUTPUT_VARIABLE outVar
   )
endmacro()
