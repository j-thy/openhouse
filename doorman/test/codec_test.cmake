# ctest doorman_codec: dumps the frames a capture's devices sent (a replay with LH_AIR_DUMP) and checks
# every one survives the batch codec exactly, at the answer size and poll period of 8 devices.
set(ENV{LH_AIR} base)
set(ENV{LH_AIR_DUMP} ${FRAMES})
execute_process(COMMAND ${LHTOOL} session ${CAPTURE} OUTPUT_QUIET ERROR_QUIET RESULT_VARIABLE replay)
if(NOT replay EQUAL 0)
  message(FATAL_ERROR "replay failed (${replay})")
endif()
unset(ENV{LH_AIR})
unset(ENV{LH_AIR_DUMP})
execute_process(COMMAND ${ROUNDTRIP} ${FRAMES} 10 176 RESULT_VARIABLE roundtrip)
if(NOT roundtrip EQUAL 0)
  message(FATAL_ERROR "frames changed going through the codec")
endif()
