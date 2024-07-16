#include <iostream>
#include <vector>
#include <queue>
#include <limits>
#include <algorithm>

class Graph {
private:
    int V; // Number of vertices
    std::vector<std::vector<std::pair<int, int>>> adj; // Adjacency list

public:
    Graph(int vertices) : V(vertices) {
        adj.resize(V);
    }

    void addEdge(int u, int v, int weight) {
        adj[u].push_back({v, weight});
        adj[v].push_back({u, weight}); // For undirected graph
    }

    // Dijkstra's Algorithm
    std::vector<int> dijkstra(int src) {
        std::vector<int> dist(V, std::numeric_limits<int>::max());
        std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, std::greater<std::pair<int, int>>> pq;

        dist[src] = 0;
        pq.push({0, src});

        while (!pq.empty()) {
            int u = pq.top().second;
            pq.pop();

            for (auto& neighbor : adj[u]) {
                int v = neighbor.first;
                int weight = neighbor.second;

                if (dist[v] > dist[u] + weight) {
                    dist[v] = dist[u] + weight;
                    pq.push({dist[v], v});
                }
            }
        }

        return dist;
    }

    // Bellman-Ford Algorithm
    std::vector<int> bellmanFord(int src) {
        std::vector<int> dist(V, std::numeric_limits<int>::max());
        dist[src] = 0;

        for (int i = 1; i <= V - 1; i++) {
            for (int u = 0; u < V; u++) {
                for (auto& neighbor : adj[u]) {
                    int v = neighbor.first;
                    int weight = neighbor.second;
                    if (dist[u] != std::numeric_limits<int>::max() && dist[u] + weight < dist[v]) {
                        dist[v] = dist[u] + weight;
                    }
                }
            }
        }

        return dist;
    }

    // Add implementations for Kruskal's, Prim's, and Tarjan's algorithms here
};

int main() {
    Graph g(5);
    g.addEdge(0, 1, 4);
    g.addEdge(0, 2, 8);
    g.addEdge(1, 2, 2);
    g.addEdge(1, 3, 5);
    g.addEdge(2, 3, 5);
    g.addEdge(2, 4, 9);
    g.addEdge(3, 4, 4);

    std::vector<int> dijkstraResult = g.dijkstra(0);
    std::cout << "Dijkstra's Algorithm Result:\n";
    for (int i = 0; i < dijkstraResult.size(); i++) {
        std::cout << "Distance from source to vertex " << i << ": " << dijkstraResult[i] << "\n";
    }

    std::vector<int> bellmanFordResult = g.bellmanFord(0);
    std::cout << "\nBellman-Ford Algorithm Result:\n";
    for (int i = 0; i < bellmanFordResult.size(); i++) {
        std::cout << "Distance from source to vertex " << i << ": " << bellmanFordResult[i] << "\n";
    }

    return 0;
}
