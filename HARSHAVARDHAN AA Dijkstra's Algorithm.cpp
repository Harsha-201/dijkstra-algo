#include <iostream>
#include <vector>
#include <unordered_map>
#include <set>
#include <limits>
#include <string>
#include <cctype> // For toupper

using namespace std;

// Function to find the vertex with the minimum distance value, from
// the set of vertices not yet included in the shortest path tree
char minDistance(unordered_map<char, int>& dist, set<char>& visited) {
    // Initialize min value
    int min = numeric_limits<int>::max();
    char min_vertex = '\0';

    for (const auto& pair : dist) {
        if (visited.find(pair.first) == visited.end() && pair.second <= min) {
            min = pair.second;
            min_vertex = pair.first;
        }
    }
    return min_vertex;
}

// Function that implements Dijkstra's single source shortest path algorithm
// for a graph represented using adjacency list representation
void dijkstra(unordered_map<char, vector<pair<char, int>>>& graph, char src, char dest) {
    // Initialize the distances with infinity and set of visited nodes
    unordered_map<char, int> dist;
    unordered_map<char, char> parent;  // To store the path
    set<char> visited;

    for (const auto& node : graph) {
        dist[node.first] = numeric_limits<int>::max();
    }

    // Distance to the source is 0
    dist[src] = 0;
    parent[src] = '\0';  // No parent for the source

    // Find shortest path for all vertices
    for (size_t i = 0; i < graph.size(); ++i) {
        // Pick the minimum distance vertex from the set of vertices not yet processed
        char u = minDistance(dist, visited);

        // Mark the picked vertex as processed
        if (u == '\0') break; // If no vertex is found, break the loop
        visited.insert(u);

        // Update dist value of the adjacent vertices of the picked vertex
        for (const auto& neighbor : graph[u]) {
            char v = neighbor.first;
            int weight = neighbor.second;

            // Update dist[v] only if not in visited, there is an edge from u to v,
            // and total weight of path from src to v through u is smaller than the current value of dist[v]
            if (visited.find(v) == visited.end() && dist[u] != numeric_limits<int>::max() && dist[u] + weight < dist[v]) {
                dist[v] = dist[u] + weight;
                parent[v] = u;
            }
        }
    }

    // Print the constructed distance array
    cout << "Vertex\t\tDistance from Source\tPath" << endl;
    for (const auto& d : dist) {
        cout << d.first << "\t\t" << d.second << "\t\t";

        // Print the path
        string path = "";
        char step = d.first;
        while (step != '\0') {
            path = step + path;
            step = parent[step];
        }
        cout << path << endl;
    }

    // Print the shortest path and its cost for the destination node
    cout << "\nShortest path from " << src << " to " << dest << ": ";
    if (dist[dest] == numeric_limits<int>::max()) {
        cout << "No path exists" << endl;
    } else {
        // Backtrack from destination to source to print the path
        string shortest_path = "";
        char step = dest;
        while (step != '\0') {
            shortest_path = step + shortest_path;
            step = parent[step];
        }
        cout << shortest_path << endl;
        cout << "Cost: " << dist[dest] << endl;
    }
}

int main() {
    // Create the graph as an adjacency list
    unordered_map<char, vector<pair<char, int>>> graph;

    // Graph definition
    graph['A'] = {{'B', 10}, {'E', 3}};
    graph['B'] = {{'C', 2}, {'E', 4}};
    graph['C'] = {{'D', 9}};
    graph['D'] = {{'C', 7}};
    graph['E'] = {{'B', 1}, {'C', 8}};

    char start, end;
    cout << "Enter the starting node: ";
    cin >> start;
    cout << "Enter the ending node: ";
    cin >> end;

    // Convert input to uppercase to handle case-insensitive input
    start = toupper(start);
    end = toupper(end);

    // Check if the nodes exist in the graph (case-sensitive)
    if (graph.find(start) == graph.end() || graph.find(end) == graph.end()) {
        cout << "Error: One or both of the nodes do not exist in the graph." << endl;
        return 1;
    }

    // Run Dijkstra's algorithm
    dijkstra(graph, start, end);

    return 0;
}
