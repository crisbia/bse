set(BSE_SRC_DIR
    Source
)

set(BSE_HDR_DIR
    Include
)

set(BSE_PHYSICS_SRC
    ${BSE_SRC_DIR}/bseBody.cpp
    ${BSE_SRC_DIR}/bseCollision.cpp
    ${BSE_SRC_DIR}/bseDynamics.cpp
    ${BSE_SRC_DIR}/bseIsland.cpp
    ${BSE_SRC_DIR}/bseMaterial.cpp
    ${BSE_SRC_DIR}/bseSAP.cpp
    ${BSE_SRC_DIR}/bseScene.cpp
    ${BSE_SRC_DIR}/bseShape.cpp
)

add_library(bsePhysics STATIC
    ${BSE_PHYSICS_SRC}
)
target_include_directories(bsePhysics PUBLIC 
    ${BSE_HDR_DIR}
    ../bseCommons/Include)
