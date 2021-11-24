set(BSE_SRC_DIR
    Source
)

set(BSE_HDR_DIR
    Include
)

set(BSE_COMMONS_SRC
    ${BSE_SRC_DIR}/bseMath.cpp
    ${BSE_SRC_DIR}/bseMemoryManager.cpp
    ${BSE_SRC_DIR}/bseProfilingTools.cpp
    ${BSE_SRC_DIR}/bseTask.cpp
    ${BSE_SRC_DIR}/bseThreadedContexts.cpp
    ${BSE_SRC_DIR}/bseThreadingTools.cpp
)

add_library(bseCommons STATIC
    ${BSE_COMMONS_SRC}
)
target_include_directories(bseCommons PUBLIC 
    ${BSE_HDR_DIR})
