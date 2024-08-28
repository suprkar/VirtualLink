#include <iostream>
#include <vector>
#include <queue>
#include <stack>
#include <limits>
#include <algorithm>
#include <random>
#include <functional>
#include <SDL2/SDL.h>

using namespace std;

const int WindowWidth = 800;
const int WindowHeight = 600;
const int VertexRadius = 10;
const int EdgeThickness = 2;

struct Vertex {
    int x, y;
};

struct Edge {
    int source, destination, weight;
};

class Graph {
private:
    int numVertices;
    vector<Vertex> vertices;
    vector<vector<pair<int, int>>> adjacencyList;
    vector<Edge> edges;
    bool isDirected;

public:
    Graph(int vertices, bool directed = false) : numVertices(vertices), isDirected(directed) {
        adjacencyList.resize(numVertices);
    }

    void addVertex(int x, int y) {
        vertices.push_back({x, y});
    }

    void addEdge(int source, int destination, int weight) {
        adjacencyList[source].push_back({destination, weight});
        edges.push_back({source, destination, weight});
        if (!isDirected) {
            adjacencyList[destination].push_back({source, weight});
        }
    }

    void generateRandomGraph(int numEdges) {
        random_device rd;
        mt19937 gen(rd());
        uniform_int_distribution<int> disX(0, WindowWidth);
        uniform_int_distribution<int> disY(0, WindowHeight);
        uniform_int_distribution<int> disWeight(1, 20);

        vertices.clear();
        adjacencyList.clear();
        edges.clear();
        adjacencyList.resize(numVertices);

        for (int i = 0; i < numVertices; ++i) {
            addVertex(disX(gen), disY(gen));
        }

        for (int i = 0; i < numEdges; ++i) {
            int source = i % numVertices;
            int destination = (i + 1) % numVertices;
            int weight = disWeight(gen);
            addEdge(source, destination, weight);
        }
    }

    void drawGraph(SDL_Renderer* renderer) {
        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
        SDL_RenderClear(renderer);

        for (const auto& edge : edges) {
            SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
            SDL_RenderDrawLine(renderer, vertices[edge.source].x, vertices[edge.source].y,
                               vertices[edge.destination].x, vertices[edge.destination].y);
        }

        for (const auto& vertex : vertices) {
            SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
            SDL_Rect rect = {vertex.x - (VertexRadius / 2), vertex.y - (VertexRadius / 2), VertexRadius, VertexRadius};
            SDL_RenderFillRect(renderer, &rect);
        }

        SDL_RenderPresent(renderer);
    }

    void drawEdge(SDL_Renderer* renderer, int source, int destination, SDL_Color color) {
        SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
        SDL_RenderDrawLine(renderer, vertices[source].x, vertices[source].y,
                           vertices[destination].x, vertices[destination].y);
        SDL_RenderPresent(renderer);
    }

    void drawVertex(SDL_Renderer* renderer, int vertex, SDL_Color color) {
        SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
        SDL_Rect rect = {vertices[vertex].x - (VertexRadius / 2), vertices[vertex].y - (VertexRadius / 2), VertexRadius, VertexRadius};
        SDL_RenderFillRect(renderer, &rect);
        SDL_RenderPresent(renderer);
    }

    void dijkstra(SDL_Renderer* renderer, int start) {
        vector<int> dist(numVertices, numeric_limits<int>::max());
        vector<bool> visited(numVertices, false);
        priority_queue<pair<int, int>, vector<pair<int, int>>, greater<>> pq;

        dist[start] = 0;
        pq.push({0, start});

        while (!pq.empty()) {
            int u = pq.top().second;
            pq.pop();

            if (visited[u]) continue;
            visited[u] = true;

            SDL_Color vertexColor = {0, 255, 0, 255};
            drawVertex(renderer, u, vertexColor);

            for (const auto& edge : adjacencyList[u]) {
                int v = edge.first;
                int weight = edge.second;
                if (!visited[v] && dist[u] + weight < dist[v]) {
                    dist[v] = dist[u] + weight;
                    pq.push({dist[v], v});
                    
                    SDL_Color edgeColor = {0, 0, 255, 255};
                    drawEdge(renderer, u, v, edgeColor);
                    SDL_Delay(500);
                }
            }
        }
    }

    void kruskal(SDL_Renderer* renderer) {
        vector<int> parent(numVertices);
        for (int i = 0; i < numVertices; ++i)
            parent[i] = i;

        sort(edges.begin(), edges.end(), [](const Edge& a, const Edge& b) {
            return a.weight < b.weight;
        });

        auto find = [&parent](int i) {
            while (i != parent[i])
                i = parent[i];
            return i;
        };

        auto unionSets = [&parent, &find](int u, int v) {
            int set_u = find(u);
            int set_v = find(v);
            parent[set_u] = set_v;
        };

        for (const auto& edge : edges) {
            int u = find(edge.source);
            int v = find(edge.destination);

            if (u != v) {
                unionSets(u, v);

                SDL_Color edgeColor = {255, 165, 0, 255};
                drawEdge(renderer, edge.source, edge.destination, edgeColor);
                SDL_Delay(500);
            }
        }
    }

    void bellmanFord(SDL_Renderer* renderer, int start) {
        vector<int> dist(numVertices, numeric_limits<int>::max());
        dist[start] = 0;

        for (int i = 0; i < numVertices - 1; ++i) {
            for (const auto& edge : edges) {
                if (dist[edge.source] != numeric_limits<int>::max() &&
                    dist[edge.source] + edge.weight < dist[edge.destination]) {
                    dist[edge.destination] = dist[edge.source] + edge.weight;

                    SDL_Color edgeColor = {0, 0, 255, 255};
                    drawEdge(renderer, edge.source, edge.destination, edgeColor);
                    SDL_Delay(500);
                }
            }
        }

        // Check for negative-weight cycles
        for (const auto& edge : edges) {
            if (dist[edge.source] != numeric_limits<int>::max() &&
                dist[edge.source] + edge.weight < dist[edge.destination]) {
                cout << "Graph contains a negative-weight cycle\n";
                return;
            }
        }

        // Highlight the shortest paths
        for (int i = 0; i < numVertices; ++i) {
            if (i != start && dist[i] != numeric_limits<int>::max()) {
                SDL_Color vertexColor = {0, 255, 0, 255};
                drawVertex(renderer, i, vertexColor);
            }
        }
    }

    void tarjan(SDL_Renderer* renderer) {
        vector<int> disc(numVertices, -1);
        vector<int> low(numVertices, -1);
        vector<bool> stackMember(numVertices, false);
        stack<int> st;

        int time = 0;

        function<void(int)> tarjanUtil = [&](int u) {
            disc[u] = low[u] = ++time;
            st.push(u);
            stackMember[u] = true;

            for (const auto& v : adjacencyList[u]) {
                int v_vertex = v.first;
                if (disc[v_vertex] == -1) {
                    tarjanUtil(v_vertex);
                    low[u] = min(low[u], low[v_vertex]);
                } else if (stackMember[v_vertex]) {
                    low[u] = min(low[u], disc[v_vertex]);
                }
            }

            if (low[u] == disc[u]) {
                vector<int> component;
                while (true) {
                    int v = st.top();
                    st.pop();
                    stackMember[v] = false;
                    component.push_back(v);
                    if (u == v) break;
                }

                // Highlight the strongly connected component
                SDL_Color componentColor = {
                    static_cast<Uint8>(rand() % 256),
                    static_cast<Uint8>(rand() % 256),
                    static_cast<Uint8>(rand() % 256),
                    255
                };
                for (int v : component) {
                    drawVertex(renderer, v, componentColor);
                }
                SDL_RenderPresent(renderer);
                SDL_Delay(1000);
            }
        };

        for (int i = 0; i < numVertices; ++i) {
            if (disc[i] == -1) {
                tarjanUtil(i);
            }
        }
    }

    void algorithmMenu(SDL_Renderer* renderer) {
        bool quit = false;
        SDL_Event event;

        while (!quit) {
            while (SDL_PollEvent(&event)) {
                if (event.type == SDL_QUIT) {
                    quit = true;
                } else if (event.type == SDL_KEYDOWN) {
                    switch (event.key.keysym.sym) {
                        case SDLK_1:
                            drawGraph(renderer);
                            dijkstra(renderer, 0);
                            break;
                        case SDLK_2:
                            drawGraph(renderer);
                            kruskal(renderer);
                            break;
                        case SDLK_3:
                            drawGraph(renderer);
                            bellmanFord(renderer, 0);
                            break;
                        case SDLK_4:
                            drawGraph(renderer);
                            tarjan(renderer);
                            break;
                        case SDLK_q:
                            quit = true;
                            break;
                    }
                }
            }

            drawGraph(renderer);
            SDL_RenderPresent(renderer);
        }
    }
};

void initializeSDL(SDL_Window*& window, SDL_Renderer*& renderer) {
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        cerr << "SDL initialization failed: " << SDL_GetError() << endl;
        exit(1);
    }

    if (SDL_CreateWindowAndRenderer(WindowWidth, WindowHeight, 0, &window, &renderer) != 0) {
        cerr << "Window or renderer creation failed: " << SDL_GetError() << endl;
        SDL_Quit();
        exit(1);
    }

    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderClear(renderer);
    SDL_RenderPresent(renderer);
}

void displayMenu() {
    cout << "\nGraph Algorithm Visualization\n";
    cout << "1. Run Dijkstra's Algorithm\n";
    cout << "2. Run Kruskal's Algorithm\n";
    cout << "3. Run Bellman-Ford Algorithm\n";
    cout << "4. Run Tarjan's Algorithm\n";
    cout << "5. Generate New Random Graph\n";
    cout << "6. Quit\n";
    cout << "Enter your choice: ";
}

int main(int argc, char* argv[]) {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    Graph graph(10);
    graph.generateRandomGraph(15);

    int choice;
    bool quit = false;

    while (!quit) {
        displayMenu();
        cin >> choice;

        switch (choice) {
            case 1:
            case 2:
            case 3:
            case 4:
                if (!window) {
                    initializeSDL(window, renderer);
                }
                graph.drawGraph(renderer);
                if (choice == 1) {
                    graph.dijkstra(renderer, 0);
                } else if (choice == 2) {
                    graph.kruskal(renderer);
                } else if (choice == 3) {
                    graph.bellmanFord(renderer, 0);
                } else {
                    graph.tarjan(renderer);
                }
                cout << "Press Enter to continue...";
                cin.ignore();
                cin.get();
                break;
            case 5:
                graph = Graph(10, true);  // Create a directed graph for Tarjan's algorithm
                graph.generateRandomGraph(15);
                cout << "New random directed graph generated.\n";
                break;
            case 6:
                quit = true;
                break;
            default:
                cout << "Invalid choice. Please try again.\n";
        }

        if (window) {
            SDL_DestroyRenderer(renderer);
            SDL_DestroyWindow(window);
            window = nullptr;
            renderer = nullptr;
        }
    }

    SDL_Quit();
    return 0;
}
