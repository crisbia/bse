set(BSE_SRC_DIR
    Source
)

set(BSE_HDR_DIR
    Include
)

set(BSE_TOOLS_SRC
    ${BSE_SRC_DIR}/bseTools.cpp
)

add_library(bseTools STATIC
    ${BSE_TOOLS_SRC}
)
target_include_directories(bseTools PUBLIC 
    ${BSE_HDR_DIR}
    ../bseCommons/Include
    ../bsePhysics/Include)
