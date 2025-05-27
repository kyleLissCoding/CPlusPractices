// HW3 Compute the minimum spanning tree for an inputted graph
// Kyle Liss
// 5/26/2025

# include <iostream>
# include <vector>
# include <fstream>
# include <string>
# include <sstream>
# include <algorithm>
# include <list>

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

    // Constructor that reads graph from file
    Graph(string filename) {
        ifstream file(filename);
        if (!file.is_open()) {
            cout << "Error opening file" << endl;
            return;
        }

        // Read number of nodes
        file >> numNodes;
        
        // Initialize adjacency matrix
        adjacencyMatrix.resize(numNodes, vector<double>(numNodes, 0.0));
        numEdges = 0;
        
        // Read edges (i, j, cost)
        int i, j;
        double cost;
        while (file >> i >> j >> cost) {
            adjacencyMatrix[i][j] = cost;
            adjacencyMatrix[j][i] = cost;  // Since it's undirected
            numEdges++;
        }
        
        file.close();
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

class MST {
    private:
        Graph graph;
        vector<pair<int, int>> mstEdges;
        double totalCost;

        // Union-Find data structure
        class UnionFind {
        private:
            vector<int> parent;
            vector<int> rank;
        public:
            UnionFind(int n) {
                parent.resize(n);
                rank.resize(n, 0);
                for(int i = 0; i < n; i++) parent[i] = i;
            }
            
            int find(int x) {
                if(parent[x] != x) parent[x] = find(parent[x]);
                return parent[x];
            }
            
            void unite(int x, int y) {
                int px = find(x), py = find(y);
                if(px == py) return;
                if(rank[px] < rank[py]) parent[px] = py;
                else if(rank[px] > rank[py]) parent[py] = px;
                else {
                    parent[py] = px;
                    rank[px]++;
                }
            }
        };

    public:
        MST(Graph g) : graph(g), totalCost(0) {}

        void kruskal() {
            // Get all edges
            vector<tuple<int, int, double>> edges;
            for(int i = 0; i < graph.getNumNodes(); i++) {
                for(int j = i + 1; j < graph.getNumNodes(); j++) {
                    if(graph.isConnected(i, j)) {
                        edges.push_back({i, j, graph.getEdgeValue(i, j)});
                    }
                }
            }

            // Sort edges by weight
            sort(edges.begin(), edges.end(), 
                [](const tuple<int, int, double>& a, const tuple<int, int, double>& b) {
                    return get<2>(a) < get<2>(b);
                });

            UnionFind uf(graph.getNumNodes());
            mstEdges.clear();
            totalCost = 0;

            // Build MST
            for(const auto& edge : edges) {
                int from = get<0>(edge);
                int to = get<1>(edge);
                double weight = get<2>(edge);

                if(uf.find(from) != uf.find(to)) {
                    mstEdges.push_back({from, to});
                    totalCost += weight;
                    uf.unite(from, to);
                }
            }
        }

        void printMST() {
            cout << "MST Edges:" << endl;
            for(const auto& edge : mstEdges) {
                cout << edge.first << " - " << edge.second 
                    << " (weight: " << graph.getEdgeValue(edge.first, edge.second) << ")" << endl;
            }
            cout << "Total cost: " << totalCost << endl;
        }
};

int main() {
    // Test with file input
    Graph g("graph.txt");
    
    MST mst(g);
    mst.kruskal();
    mst.printMST();
    
    return 0;
}