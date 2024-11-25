#include <iostream>
#include <vector>
#include <unordered_map>
#include <list>
#include <stack>
#include <queue>

using namespace std;

const int SIZE = 12; // Updated size to 11 NOTE: LLM caused exception. Manually edited to 12.

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
            cout << i << " --> ";
            for (Pair v : adjList[i])
                cout << "(" << v.first << ", " << v.second << ") ";
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
};

int main() {
    // Creates a vector of graph edges/weights
    vector<Edge> edges = {
        // (x, y, w) —> edge from x to y having weight w
        {0,1,10},{0,2,15},{0,3,20},{2,3,25},{2,4,30},{4,7,35},{7,8,40},{8,9,45},{9,10,50},{10,11,55},{11,2,60}
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

    return 0;
}
