cmake_minimum_required (VERSION 2.6)
project (segment_transfer)

set (CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")


set (BIN_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../bin") 
set (LIB_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../lib") 
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BIN_PATH})

add_definitions (-DMSH_NO_WINDOW)

include_directories ("${LIB_PATH}")
include_directories ("${LIB_PATH}/rs")
include_directories ("${LIB_PATH}/gco")
include_directories ("${LIB_PATH}/im3d")


list (APPEND SOURCE_LIST arrangement_optimization.cpp database_update.cpp main.cpp ${LIB_PATH}/rs/rs_pointcloud_filters.cpp)
list (APPEND SOURCE_LIST ${LIB_PATH}/gco/GCOptimization.cpp ${LIB_PATH}/gco/LinkedBlockList.cpp)

add_executable (segment_transfer ${SOURCE_LIST})

if (${USE_WINDOW})
	target_link_libraries (segment_transfer glfw)
endif()


