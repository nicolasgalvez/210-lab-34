#include <iostream>
#include <vector>
#include <unordered_map>
#include <list>
#include <stack>
#include <queue>
#include <set>

using namespace std;

const int SIZE = 12; // Updated size to 11 NOTE: LLM caused exception. Manually edited to 12.

const string systemNames[SIZE] = {
    "Earth", "Vulcan", "Andoria", "Qo'noS", "Romulus", "Cardassia", "Bajor", "Ferenginar", "Betazed", "Risa", "Trill", "Tellar"
};

struct Edge {
    int src, dest, weight;
};

typedef pair<int, int> Pair;  // Creates alias 'Pair' for the pair<int,int> data type

class Graph {
public:
    // a vector of vectors of Pairs to represent an adjacency list
    vector<vector<Pair>> adjList;

    // Graph Constructor
    Graph(vector<Edge> const &edges) {
        // resize the vector to hold SIZE elements of type vector<Edge>
        adjList.resize(SIZE);

        // add edges to the directed graph
        for (auto &edge: edges) {
            int src = edge.src;
            int dest = edge.dest;
            int weight = edge.weight;

            // insert at the end
            adjList[src].push_back(make_pair(dest, weight));
            // for an undirected graph, add an edge from dest to src also
            adjList[dest].push_back(make_pair(src, weight));
        }
    }

    // Print the graph's adjacency list
    void printGraph() {
        cout << "Graph's adjacency list:" << endl;
        for (int i = 0; i < adjList.size(); i++) {
            cout << systemNames[i] << " (" << i << ") --> ";
            for (Pair v : adjList[i])
                cout << "(" << systemNames[v.first] << " (" << v.first << "), Dilithium: " << v.second << ") ";
            cout << endl;
        }
    }

    void printStarSystem() {
        cout << "Star System Model:" << endl;
        for (int i = 0; i < adjList.size(); i++) {
            cout << "Planet " << systemNames[i] << " (" << i << ") warps to: ";
            for (Pair v : adjList[i])
                cout << "Planet " << systemNames[v.first] << " (" << v.first << ") (Dilithium: " << v.second << ") ";
            cout << endl;
        }
    }

    // Function to perform DFS on the graph
    void DFS(int start) {
        unordered_map<int, bool> visited;
        stack<int> stack;
        stack.push(start);

        while (!stack.empty()) {
            int v = stack.top();
            stack.pop();

            if (!visited[v]) {
                cout << v << " ";
                visited[v] = true;
            }

            for (auto &neighbor : adjList[v]) {
                if (!visited[neighbor.first]) {
                    stack.push(neighbor.first);
                }
            }
        }
        cout << endl;
    }

    // Function to perform BFS on the graph
    void BFS(int start) {
        unordered_map<int, bool> visited;
        queue<int> queue;
        queue.push(start);
        visited[start] = true;

        while (!queue.empty()) {
            int v = queue.front();
            queue.pop();
            cout << v << " ";

            for (auto &neighbor : adjList[v]) {
                if (!visited[neighbor.first]) {
                    queue.push(neighbor.first);
                    visited[neighbor.first] = true;
                }
            }
        }
        cout << endl;
    }

    // Function to print warp map using DFS
    void plotWarpJump(int start) {
        unordered_map<int, bool> visited;
        stack<int> stack;
        stack.push(start);

        cout << "Plotting sequence of jumps from origin " << systemNames[start] << " (" << start << "):" << endl;

        while (!stack.empty()) {
            int v = stack.top();
            stack.pop();

            if (!visited[v]) {
                cout << "System " << systemNames[v] << " (" << v << ") -> ";
                visited[v] = true;
            }

            for (auto &neighbor : adjList[v]) {
                if (!visited[neighbor.first]) {
                    stack.push(neighbor.first);
                }
            }
        }
        cout << "End" << endl;
    }

    // Function to show potential jumps using BFS
    void showPotentialJumps(int start) {
        unordered_map<int, bool> visited;
        queue<int> queue;
        queue.push(start);
        visited[start] = true;

        cout << "Potential jumps from system " << systemNames[start] << " (" << start << ") (BFS):" << endl;

        while (!queue.empty()) {
            int v = queue.front();
            queue.pop();
            cout << "System " << systemNames[v] << " (" << v << ") -> ";

            for (auto &neighbor : adjList[v]) {
                if (!visited[neighbor.first]) {
                    queue.push(neighbor.first);
                    visited[neighbor.first] = true;
                }
            }
        }
        cout << "End" << endl;
    }

    // Function to find the shortest path from a node using Dijkstra's algorithm
    void findShortestPath(int start) {
        set<Pair> setds;
        vector<int> dist(SIZE, INT_MAX);
        vector<int> prev(SIZE, -1);

        setds.insert(make_pair(0, start));
        dist[start] = 0;

        while (!setds.empty()) {
            Pair tmp = *(setds.begin());
            setds.erase(setds.begin());

            int u = tmp.second;

            for (auto &neighbor : adjList[u]) {
                int v = neighbor.first;
                int weight = neighbor.second;

                if (dist[v] > dist[u] + weight) {
                    if (dist[v] != INT_MAX) {
                        setds.erase(setds.find(make_pair(dist[v], v)));
                    }
                    dist[v] = dist[u] + weight;
                    prev[v] = u;
                    setds.insert(make_pair(dist[v], v));
                }
            }
        }

        cout << "Shortest paths from node " << start << ":" << endl;
        for (int i = 0; i < SIZE; i++) {
            if (dist[i] == INT_MAX) {
                cout << start << " -> " << i << " : Infinity" << endl;
            } else {
                cout << start << " -> " << i << " : " << dist[i] << " (Path: ";
                int crawl = i;
                while (prev[crawl] != -1) {
                    cout << crawl << " <- ";
                    crawl = prev[crawl];
                }
                cout << start << ")" << endl;
            }
        }
    }

    // Function to check if the graph is connected from a given starting node
    bool isConnected(int start) {
        unordered_map<int, bool> visited;
        queue<int> queue;
        queue.push(start);
        visited[start] = true;

        while (!queue.empty()) {
            int v = queue.front();
            queue.pop();

            for (auto &neighbor : adjList[v]) {
                if (!visited[neighbor.first]) {
                    queue.push(neighbor.first);
                    visited[neighbor.first] = true;
                }
            }
        }

        for (int i = 0; i < SIZE; i++) {
            if (!visited[i]) {
                return false;
            }
        }
        return true;
    }
};


int main() {
    // Creates a vector of graph edges/weights
    vector<Edge> edges = {
        // (x, y, w) —> edge from x to y having weight w
        {0,1,10},{0,7,15},{0,6,20},{2,3,25},{5,4,30},{4,7,35},{7,8,40},{8,9,45},{9,10,50},{10,11,55},{6,2,60}
        // LLM INPUT: Change the graph by deleting at least two nodes and adding at least six nodes. Change the weights as well.
        // LLM OUTPUT: Modify the graph by deleting nodes 5 and 6, and adding nodes 7, 8, 9, 10, 11, and 12. Update the edges and weights accordingly.
    };

    // Creates graph
    Graph graph(edges);

    // Prints adjacency list representation of graph
    graph.printGraph();

    // Perform DFS starting from vertex 0
    cout << "DFS starting from vertex 0:" << endl;
    graph.DFS(0);

    // Perform BFS starting from vertex 0
    cout << "BFS starting from vertex 0:" << endl;
    graph.BFS(0);

    cout << endl << "Star System Model" << endl;
    // Prints adjacency list representation of graph
    graph.printStarSystem();

    cout << endl;
    cout << "Compute Nav Points" << endl;
    // Prints warp map using DFS starting from system 0
    graph.plotWarpJump(0);

    cout << endl << "Potential Jumps" << endl;
    // Show potential jumps using BFS starting from system 0
    graph.showPotentialJumps(0);

    cout << endl << "Shortest Paths" << endl;
    // Find the shortest path from node 0
    graph.findShortestPath(0);

    // Check if the graph is connected from node 0
    if (graph.isConnected(0)) {
        cout << "The graph is connected from node 0." << endl;
    } else {
        cout << "The graph is not connected from node 0." << endl;
    }

    return 0;
}
