# VirtualLink

This project implements various graph algorithms in C++, providing a simple console-based visualization of their operations and results.

## Algorithms Implemented

1. Dijkstra's Algorithm
2. Bellman-Ford Algorithm
3. Kruskal's Algorithm
4. Prim's Algorithm
5. Tarjan's Algorithm for Strongly Connected Components

## Features

- Create and visualize undirected weighted graphs
- Find shortest paths using Dijkstra's and Bellman-Ford algorithms
- Generate Minimum Spanning Trees using Kruskal's and Prim's algorithms
- Identify Strongly Connected Components using Tarjan's algorithm

## Requirements

- C++ compiler with C++11 support or later

## How to Use

1. Clone the repository:
  ```
git clone https://github.com/yourusername/graph-algorithm-visualizer.git
```
2. Compile the code:
```
g++ -std=c++11 main.cpp -o graph_visualizer
```
3. Run the executable:
```
./graph_visualizer
```
4. The program will create a sample graph and demonstrate all implemented algorithms.

## Customizing the Graph

To create your own graph or modify the existing one, edit the `main()` function in `main.cpp`. Use the `addEdge()` function to add edges to the graph:

```cpp
Graph g(num_vertices);
g.addEdge(source, destination, weight);
```
