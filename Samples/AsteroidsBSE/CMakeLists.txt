find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

set(BSE_ASTEROIDS_SRC
    AsteroidsBSE.cpp
    ../../Commons/DrawUtils.cpp
)

include_directories(
    ${GLUT_INCLUDE_DIR}
    ${OPENGL_INCLUDE_DIRS}
    ../../bse/bseCommons/Include
    ../../bse/bsePhysics/Include
    ../../bse/bseAI/Include
    ../../Commons
)

add_compile_definitions(GL_SILENCE_DEPRECATION)

add_executable(AsteroidsBSE
    ${BSE_ASTEROIDS_SRC}
)
target_link_libraries(AsteroidsBSE PUBLIC
    bsePhysics
    bseCommons
    ${OPENGL_LIBRARIES} 
    glfw
 )