cmake_minimum_required(VERSION 2.6)

Project(Tutorial)

include_directories("include/my_math")

add_library(MathFunctions src/my_math/mysqrt.cpp)

add_executable(Tutorial src/tutorial.cpp)

target_link_libraries(Tutorial MathFunctions)