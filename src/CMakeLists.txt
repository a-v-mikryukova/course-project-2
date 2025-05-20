cmake_minimum_required(VERSION 3.30)
project(mapf_good)

set(CMAKE_CXX_STANDARD 20)

add_executable(mapf_good
        main.cpp
        grid_graph.cpp
        prm.cpp
        path_planning.cpp
        constants.cpp
)