cmake_minimum_required (VERSION 2.6)
project (rsdb_viewer)

set (CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")

set (BIN_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../bin") 
set (LIB_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../lib") 
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BIN_PATH})

include_directories ("${LIB_PATH}")
include_directories ("${LIB_PATH}/rs")
include_directories ("${LIB_PATH}/gco")

find_package (OpenGL REQUIRED)
find_package (glfw3 3.3 REQUIRED)

list (APPEND SOURCE_LIST  main.cpp )
list (APPEND SOURCE_LIST ${LIB_PATH}/glad/glad.c )

add_executable (rsdb_viewer ${SOURCE_LIST})
target_link_libraries (rsdb_viewer glfw)

