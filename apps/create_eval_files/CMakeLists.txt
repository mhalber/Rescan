cmake_minimum_required (VERSION 2.6)
project (create_eval_files)

set (CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")
set (CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")

set (BIN_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../bin")
set (LIB_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../lib")
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BIN_PATH})

include_directories ("${LIB_PATH}")
include_directories ("${LIB_PATH}/rs")
add_executable (create_eval_files main.cpp)
