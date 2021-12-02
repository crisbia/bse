find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

set(BSE_MAZE_SRC
    main.cpp
    MazeBSE.cpp
    ../../Commons/DrawUtils.cpp
    ../../Commons/SysUtils.cpp
)

include_directories(
    ${OPENGL_INCLUDE_DIRS}
    ../../bse/bseCommons/Include
    ../../bse/bsePhysics/Include
    ../../bse/bseAI/Include
    ../../Commons
)

add_compile_definitions(GL_SILENCE_DEPRECATION)

add_executable(MazeBSE
    ${BSE_MAZE_SRC}
)
target_link_libraries(MazeBSE PUBLIC
    bsePhysics
    bseCommons
    bseAI
    ${OPENGL_LIBRARIES} 
    glfw
)