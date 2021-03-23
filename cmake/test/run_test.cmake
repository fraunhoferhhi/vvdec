cmake_minimum_required( VERSION 3.12.0 FATAL_ERROR )

get_filename_component(OUTPUT_FILE ${BITSTREAM_FILE} NAME_WLE)
set(OUTPUT_FILE ${OUTPUT_FILE}.yuv)

set(CMD ${EXECUTABLE} -b ${BITSTREAM_FILE} -dph -o ${OUTPUT_FILE})
execute_process(COMMAND ${CMD}
                COMMAND_ECHO STDOUT
                RESULT_VARIABLE CMD_RESULT)
if(CMD_RESULT)
  file(REMOVE ${OUTPUT_FILE})
  message(FATAL_ERROR "Decoding ${BITSTREAM_FILE} failed.")
endif()

file(MD5 ${OUTPUT_FILE} OUTPUT_HASH)
file(REMOVE ${OUTPUT_FILE})

STRING(REGEX REPLACE "\\.bit$" ".yuv.md5" MD5_FILE ${BITSTREAM_FILE})
file(READ ${MD5_FILE} RX_MD5_HASH LIMIT 32)
string(STRIP "${RX_MD5_HASH}" RX_MD5_HASH)
string(TOLOWER "${RX_MD5_HASH}" RX_MD5_HASH)

if(NOT ${OUTPUT_HASH} STREQUAL ${RX_MD5_HASH})
  message(FATAL_ERROR "Output file MD5 does not match the hash provided with the bitstream. ${OUTPUT_HASH} != ${RX_MD5_HASH}")
endif()
