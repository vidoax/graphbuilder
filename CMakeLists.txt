cmake_minimum_required(VERSION 3.30)
project(GraphicBuilder)

set(CMAKE_CXX_STANDARD 26)

add_executable(GraphicBuilder main.cpp
        src/DiGraph.cpp
        src/DiGraph.h
        src/Node.h
        src/Edge.h
        src/menu.cpp
        src/menu.h
        src/DotGraph.h
        src/PriorityQueue.h
        src/GraphVisualizer.h
        src/DotGraph.cpp
)
