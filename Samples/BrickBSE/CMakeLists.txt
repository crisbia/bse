find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

set(BSE_BRICK_SRC
    BrickBSE.cpp
    ../../Commons/DrawUtils.cpp
)

include_directories(
    ${OPENGL_INCLUDE_DIRS}
    ../../bse/bseCommons/Include
    ../../bse/bsePhysics/Include
    ../../bse/bseAI/Include
    ../../Commons
)

add_compile_definitions(GL_SILENCE_DEPRECATION)

add_executable(BrickBSE
    ${BSE_BRICK_SRC}
)
target_link_libraries(BrickBSE PUBLIC
    bsePhysics
    bseCommons
    ${OPENGL_LIBRARIES} 
    glfw
)