# 🧠 GraphBuilder

**GraphBuilder** is a C++ command-line application that lets you interactively create, manipulate, and visualize graphs. It supports features like node/edge management, shortest path calculations (Dijkstra's algorithm
), DOT (Graphviz) output, and graph persistence.

---

## 📋 Features

Use GraphBuilder to:

1.   Add a node  
2.   Add an edge  
3.   Show all nodes  
4.   Show edges of a node  
5.   Show neighbors of a node  
6.   Remove a node  
7.   Remove an edge  
8.   Delete the entire graph  
9.   Modify an edge  
10.  Show the graph in [DOT (Graphviz)](https://graphviz.org/) format  
11.  Find the shortest path between nodes  
12.  Save graph to a file  
13.  Load graph from a file  
14.  Run a test driver  
15.  Exit the application  

---

## 🛠 Build & Run

### 🔧 Requirements

- C++ compiler with support for C++17 or newer
- [CMake](https://cmake.org/) 3.16+

### 🧪 Building the Project

From the root directory:

```bash
mkdir -p build
cd build
cmake ..
make
./main.cpp
