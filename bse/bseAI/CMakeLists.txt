set(BSE_SRC_DIR
    Source
)

set(BSE_HDR_DIR
    Include
)

set(BSE_AI_SRC
    ${BSE_SRC_DIR}/bseAIContext.cpp
    ${BSE_SRC_DIR}/bseAIScene.cpp
    ${BSE_SRC_DIR}/bseGraph.cpp
    ${BSE_SRC_DIR}/bseSteeringBase.cpp
)

add_library(bseAI STATIC
    ${BSE_AI_SRC}
)
target_include_directories(bseAI PUBLIC 
    ${BSE_HDR_DIR}
    ../bseCommons/Include)