cmake_minimum_required(VERSION 3.8)
project(Sokoban_solver)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES main.cpp)
file(GLOB SOURCES
        *.h
        *.hpp
        *.cpp
        )

add_executable(Sokoban_solver ${SOURCE_FILES} ${SOURCES})