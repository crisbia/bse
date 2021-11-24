cmake_minimum_required(VERSION 3.10)

# set the project name
project(BSETools)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_subdirectory(bse)

add_subdirectory(Samples)
add_subdirectory(BSEDemo)