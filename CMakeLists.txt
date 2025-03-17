cmake_minimum_required(VERSION 2.6.4)
project(CFE_SYNC_NODE_LIB C)

include_directories(fsw/public_inc)
include_directories(fsw/background)

# The shared OSAL and cFE include directories should always be used
# Note that this intentionally does NOT include PSP-specific includes, just the generic
include_directories(${CFECORE_SOURCE_DIR}/src/inc)
include_directories(${CFEPSP_SOURCE_DIR}/fsw/inc)

aux_source_directory(fsw/src LIB_SRC_FILES)

# Create the app module
add_cfe_app(sync_node_lib ${LIB_SRC_FILES})

