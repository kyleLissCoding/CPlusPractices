// HW2
// Name: Kyle Liss
// 5/21/2025
// Dijsktra's Algorithm and Graph homework

#include <iostream>
#include <vector>
#include <queue>
#include <limits>


using namespace std;

// Graph class to store helper functions, nodes, and edges as well as produce the graph itself
class Graph {
    public:

    // Sets up the number of Nodes and the density of the graph for determining the number of edges
    int numNodes;
    int numEdges;
    double density;
    // Sets up the adjacency matrix for the graph containing the primary node as i and the secondary node as j (edges)
    vector<vector<double>> adjacencyMatrix;

    // Default constructor with default values
    Graph() {
        this->numNodes = 1;
        this->numEdges = 0;
        this->density = 0.1;

        // Fills the adjacency matrix with 0s
        adjacencyMatrix.resize(numNodes, vector<double>(numNodes, 0.0));
    }

    // Constructor for the Graph class with parameters numNodes and density
    Graph(int numNodes, double density) {
        this->numNodes = numNodes;
        this->numEdges = 0;
        this->density = density;

        // Fills the adjacency matrix with 0s
        adjacencyMatrix.resize(numNodes, vector<double>(numNodes, 0.0));
        
    }

    // Generates the graph fully by taking the pre-filled adjacency matrix and adding edges based on the density
    void generateGraph(double density, double distance1, double distance2) {
        // resets the density for the graph object
        this->density = density;
        
        for(int i = 0; i < numNodes; i++) {
            for(int j = i + 1; j < numNodes; j++) {
                // Generate random number between 0 and 1
                double random = static_cast<double>(rand()) / RAND_MAX;
                
                // If random number is less than density, add the edge
                if(random < density) {
                    double weight = distance1 + (rand() % static_cast<int>((distance2 - distance1) * 10)) / 10.0;  // Random weight between 1.0 and 10.0
                    adjacencyMatrix[i][j] = weight;
                    adjacencyMatrix[j][i] = weight;
                    this->numEdges++;
                }
            }
        }
    }

    // Returns the neighbors of a node
    vector<int> neighbors(int node) {
        vector<int> neighbors;
        for(int i = 0; i < numNodes; i++) {
            if(adjacencyMatrix[node][i] > 0) {
                neighbors.push_back(i);
            }
        }
        return neighbors;
    }

    bool isConnected(int node1, int node2) {
        return adjacencyMatrix[node1][node2] > 0;
    }

    // Returns the adjacency matrix
    vector<vector<double>> getAdjacencyMatrix() {
        return adjacencyMatrix;
    }

    // Returns the number of edges
    int getNumEdges() {
        return numEdges;
    }

    // Returns the number of nodes
    int getNumNodes() {
        return numNodes;
    }
    
    // Manually adds an edge between two nodes if the edge does not exist already
    void addEdge(int i, int j, double value) {
        // making sure the edge does not exist already
        if(adjacencyMatrix[i][j] == 0) {
            adjacencyMatrix[i][j] = value;
        }
    }

    // Deletes an edge between two nodes
    void deleteEdge(int i, int j) {
        // making sure the edge exists
        if(adjacencyMatrix[i][j] > 0) {
            adjacencyMatrix[i][j] = 0;
        }
    }

    // Gets the value of an edge between two nodes
    double getEdgeValue(int i, int j) {
        return adjacencyMatrix[i][j];
    }

    // Sets the value of an edge between two nodes
    void setEdgeValue(int i, int j, double value) {
        adjacencyMatrix[i][j] = value;
    }

    // Prints the graph
    void printGraph() {
        for(int i = 0; i < numNodes; i++) {
            for(int j = 0; j < numNodes; j++) {
                cout << adjacencyMatrix[i][j] << " ";
            }
            cout << endl;
        }
    }
};



// Dijkstra's Algorithm to find the shortest path between two nodes
class Dijkstra {
    public:
    // Graph object to store the graph
    Graph graph;
    // Start node to store the start node
    int start;
    // End node to store the end node
    int end;

    // Vector to store the distance from the start node to each node
    vector<double> distance;
    // Vector to store the previous node on the shortest path to each node
    vector<int> prev;
    // Vector to store whether each node has been visited
    vector<bool> visited;


    // Default constructor for the Dijkstra class
    Dijkstra() {
        this->graph = Graph();
        this->start = 0;
        this->end = 0;
    }

    // Constructor for the Dijkstra class with parameters graph, start, and end
    Dijkstra(Graph graph, int start, int end) {
        this->graph = graph;
        this->start = start;
        this->end = end;
    }

    // Function to run Dijkstra's Algorithm
    void run() {
        vector<double> distance(graph.getNumNodes(), numeric_limits<double>::infinity());
        vector<int> prev(graph.getNumNodes(), -1);
        vector<bool> visited(graph.getNumNodes(), false);
        
        distance[start] = 0;
        
        for(int count = 0; count < graph.getNumNodes(); count++) {
            // Find vertex with minimum distance
            int u = -1;
            // Sets the minimum distance to infinity
            double minDist = numeric_limits<double>::infinity();
            // Loops through the graph and finds the vertex with the minimum distance
            for(int v = 0; v < graph.getNumNodes(); v++) {
                // If the vertex has not been visited and the distance is less than the minimum distance, set the minimum distance to the distance and the vertex to the current vertex
                if(!visited[v] && distance[v] < minDist) {
                    minDist = distance[v];
                    u = v;
                }
            }
            
            if(u == -1) break;  // No more reachable vertices
            if(u == end) break; // Found target
            
            visited[u] = true;
            
            // Update distances of neighbors using neighbors function
            for(int v : graph.neighbors(u)) {
                // If the vertex has not been visited, update the distance and previous vertex
                if(!visited[v]) {
                    double alt = distance[u] + graph.getEdgeValue(u, v);
                    // If the alternative distance is less than the current distance, update the distance and previous vertex
                    if(alt < distance[v]) {
                        distance[v] = alt;
                        prev[v] = u;
                    }
                }
            }
        }
        // Sets the distance, previous, and visited vectors to the values of the distance, previous, and visited vectors
        this->distance = distance;
        this->prev = prev;
        this->visited = visited;
    }

    // Function to print the average distance of the shortest path
    void printAverageDistance() {

        // Sets up the total path and the count of the number of paths
        double totalPath = 0.0;
        int count = 0;

        // Loops through the graph and runs Dijkstra's Algorithm for each node as the end node
        for(int end = 1; end < graph.getNumNodes(); end++) {
            Dijkstra d(graph, 0, end);
            d.run();
            // If the distance is not infinity, add the distance to the total path and increment the count
            if(d.distance[end] != numeric_limits<double>::infinity()) {
                totalPath += d.distance[end];
                count++;
            }
        }
        // Calculates the average distance of the shortest path
        double avgPath = totalPath / count;
        // Prints the average distance of the shortest path
        cout << "Average distance: " << avgPath << endl;
    }
};

// Main function to call and test the Graph class
int main() {
    // Seed the random number generator
    srand(time(0));

    // Creates the first graph with a density of 20% and a distance range of 1.0 to 10.0
    Graph graph = Graph(50, .2);
    graph.generateGraph(.2, 1.0, 10.0);
    //graph.printGraph(); // uncomment to print the graph

    // Runs Dijkstra's Algorithm for the first graph
    Dijkstra dijkstra1 = Dijkstra(graph, 0, 49);
    dijkstra1.printAverageDistance();

    // Creates the second graph with a density of 40% and a distance range of 1.0 to 10.0
    Graph graph2 = Graph(50, .4);
    graph2.generateGraph(.4, 1.0, 10.0);
    //graph2.printGraph(); // uncomment to print the graph

    // Runs Dijkstra's Algorithm for the second graph
    Dijkstra dijkstra2 = Dijkstra(graph2, 0, 49);
    dijkstra2.printAverageDistance();
    
    return 0;
};
