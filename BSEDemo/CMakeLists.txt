find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

set(BSE_DEMO_SRC
    Main.cpp
    BSEDemo.cpp
    PhysicsTests.cpp
    SettingsStack.cpp
    TestFramework.cpp
#    tests.cpp
    TestUtils.cpp
    ../Commons/DrawUtils.cpp
    ../Commons/SysUtils.cpp
)

include_directories(
    ${OPENGL_INCLUDE_DIRS}
    ../bse/bseCommons/Include
    ../bse/bsePhysics/Include
    ../bse/bseAI/Include
    ../bse/bseTools/Include
    ../Commons
)

add_compile_definitions(GL_SILENCE_DEPRECATION)

add_executable(BSEDemo
    ${BSE_DEMO_SRC}
)
target_link_libraries(BSEDemo PUBLIC
    bsePhysics
    bseCommons
    bseAI
    bseTools
    ${OPENGL_LIBRARIES} 
    glfw
)