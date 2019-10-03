# Klespsydra event code generator

macro(KpsrEventGenerator inputDir outputDir includePath disableRos disableZmq disableDds)
   MESSAGE("python3 -m kpsr_codegen -i ${inputDir} -o ${outputDir} -p ${includePath} -r ${disableRos} -z ${disableZmq} -d ${disableDds}")
   execute_process (
      COMMAND bash -c "python3 -m kpsr_codegen -i ${inputDir} -o ${outputDir} -p ${includePath} -r ${disableRos} -z ${disableZmq} -d ${disableDds}"
      OUTPUT_VARIABLE outVar
   )
endmacro()
