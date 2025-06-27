cmake_minimum_required(VERSION 2.6.4)
project(CFE_SYNC_NODE_LIB C)

include_directories(fsw/public_inc)
include_directories(fsw/backend)

# The shared OSAL and cFE include directories should always be used
# Note that this intentionally does NOT include PSP-specific includes, just the generic
include_directories(${CFECORE_SOURCE_DIR}/src/inc)
include_directories(${CFEPSP_SOURCE_DIR}/fsw/inc)

# Add these to the app CMake to import Sync_Node_Lib
# Sync-Node-Lib
#include_directories(${sync_node_lib_MISSION_DIR}/fsw/public_inc)
#include_directories(${sync_node_lib_MISSION_DIR}/fsw/backend)

aux_source_directory(fsw/src LIB_SRC_FILES)

# Create the app module
# Adds cJSON functionalty
add_cfe_app(sync_node_lib 
fsw/backend/cJSON.c
fsw/backend/cJSON.h
fsw/backend/cJSON_Utils.c
fsw/backend/cJSON_Utils.h
${LIB_SRC_FILES})

# Links the LIB-M math libraries to sync_node - not needed if on Windows OS.
IF (NOT WIN32)
  target_link_libraries(sync_node_lib m)
ENDIF()
