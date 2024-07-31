#include <iostream>
#include <vector>
#include <queue>
#include <limits>
#include <algorithm>
#include <set>

using namespace std;

class Graph {
private:
    int V; // Number of vertices
    vector<vector<pair<int, int> > > adj; // Adjacency list

    int find(vector<int>& parent, int i);
    void union_sets(vector<int>& parent, int x, int y);
    void tarjanUtil(int u, vector<int>& disc, vector<int>& low, vector<int>& st, vector<int>& stackMember, int& time);

public:
    Graph(int vertices);
    void addEdge(int u, int v, int weight);
    vector<int> dijkstra(int src);
    vector<int> bellmanFord(int src);
    vector<pair<int, pair<int, int> > > kruskal();
    vector<pair<int, int> > prim();
    void tarjan();
    void visualize();
};

Graph::Graph(int vertices) : V(vertices) {
    adj.resize(V);
}

void Graph::addEdge(int u, int v, int weight) {
    adj[u].push_back(make_pair(v, weight));
    adj[v].push_back(make_pair(u, weight)); // For undirected graph
}

vector<int> 
Graph::dijkstra(int src) 
{
    vector<int> dist(V, numeric_limits<int>::max());
    priority_queue<pair<int, int>, vector<pair<int, int> >, greater<pair<int, int> > > pq;
    dist[src] = 0;
    pq.push(make_pair(0, src));

    while (!pq.empty()) {
        int u = pq.top().second;
        pq.pop();

        for (vector<pair<int, int> >::iterator it = adj[u].begin(); it != adj[u].end(); ++it) {
            int v = it->first;
            int weight = it->second;

            if (dist[v] > dist[u] + weight) {
                dist[v] = dist[u] + weight;
                pq.push(make_pair(dist[v], v));
            }
        }
    }
    return dist;
}

vector<int> 
Graph::bellmanFord(int src) 
{
    vector<int> dist(V, numeric_limits<int>::max());
    dist[src] = 0;

    for (int i = 1; i <= V - 1; i++) {
        for (int u = 0; u < V; u++) {
            for (vector<pair<int, int> >::iterator it = adj[u].begin(); it != adj[u].end(); ++it) {
                int v = it->first;
                int weight = it->second;

                if (dist[u] != numeric_limits<int>::max() && dist[u] + weight < dist[v]) {
                    dist[v] = dist[u] + weight;
                }
            }
        }
    }
    return dist;
}

vector<pair<int, pair<int, int> > > 
Graph::kruskal() 
{
    vector<pair<int, pair<int, int> > > edges;
    for (int u = 0; u < V; u++) {
        for (vector<pair<int, int> >::iterator it = adj[u].begin(); it != adj[u].end(); ++it) {
            int v = it->first;
            int weight = it->second;
            edges.push_back(make_pair(weight, make_pair(u, v)));
        }
    }
    sort(edges.begin(), edges.end());

    vector<int> parent(V);
    for (int i = 0; i < V; i++) parent[i] = i;

    vector<pair<int, pair<int, int> > > mst;
    for (vector<pair<int, pair<int, int> > >::iterator it = edges.begin(); it != edges.end(); ++it) {
        int u = it->second.first;
        int v = it->second.second;
        int weight = it->first;

        int set_u = find(parent, u);
        int set_v = find(parent, v);

        if (set_u != set_v) {
            mst.push_back(*it);
            union_sets(parent, set_u, set_v);
        }
    }
    return mst;
}

vector<pair<int, int> > 
Graph::prim() 
{
    vector<bool> inMST(V, false);
    vector<int> key(V, numeric_limits<int>::max());
    vector<int> parent(V, -1);
    priority_queue<pair<int, int>, vector<pair<int, int> >, greater<pair<int, int> > > pq;

    int src = 0;
    pq.push(make_pair(0, src));
    key[src] = 0;

    while (!pq.empty()) {
        int u = pq.top().second;
        pq.pop();
        inMST[u] = true;

        for (vector<pair<int, int> >::iterator it = adj[u].begin(); it != adj[u].end(); ++it) {
            int v = it->first;
            int weight = it->second;

            if (!inMST[v] && key[v] > weight) {
                key[v] = weight;
                pq.push(make_pair(key[v], v));
                parent[v] = u;
            }
        }
    }

    vector<pair<int, int> > mst;
    for (int i = 1; i < V; i++) {
        mst.push_back(make_pair(parent[i], i));
    }
    return mst;
}

void 
Graph::tarjan() 
{
    vector<int> disc(V, -1), low(V, -1), stackMember(V, 0);
    vector<int> st;
    int time = 0;

    for (int i = 0; i < V; i++)
        if (disc[i] == -1)
            tarjanUtil(i, disc, low, st, stackMember, time);
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
