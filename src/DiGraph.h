#ifndef DIGRAPH_H
#define DIGRAPH_H

#include <string>
#include <unordered_map>
#include <stdexcept>
#include <fstream>

#include "GraphVisualizer.h"
#include "Node.h"
#include "PriorityQueue.h"

using namespace std;

// Forward declaration of the Edge class
class Edge;

/**
 * @class DiGraph
 * @brief Represents a directed graph.
 *
 * This class provides functionality to manage a directed graph, including adding/removing nodes and edges,
 * finding neighbors, and performing algorithms like Dijkstra's shortest path. It also supports visualization
 * through a GraphVisualizer.
 */
class DiGraph {
public:
    /**
     * @brief Destroys the DiGraph object and frees allocated memory.
     */
    ~DiGraph();

    /**
     * @brief Clears the graph by removing all nodes and edges.
     */
    void clear();

    /**
     * @brief Adds a node to the graph.
     *
     * @param key The unique identifier for the node.
     */
    void addNode(const string &key);

    /**
     * @brief Adds a directed edge between two nodes with a specified weight.
     *
     * @param key1 The key of the start node.
     * @param key2 The key of the end node.
     * @param weight The weight of the edge.
     */
    void addEdge(const string &key1, const string &key2, const float &weight);

    /**
     * @brief Modifies the weight of an existing edge.
     *
     * @param key1 The key of the start node.
     * @param key2 The key of the end node.
     * @param old_weight The current weight of the edge.
     * @param new_weight The new weight to assign to the edge.
     */
    void modifyEdgeWeight(const string& key1, const string &key2, float old_weight, float new_weight);

    /**
     * @brief Gets all nodes in the graph.
     *
     * @return A vector of pointers to all nodes in the graph.
     */
    vector<Node*> getNodes() { return nodes; }

    /**
     * @brief Gets the neighbors of a specific node.
     *
     * @param key The key of the node.
     * @return A vector of pointers to the neighboring nodes.
     */
    vector<Node*> getNeighbors(const string& key);

    /**
     * @brief Gets all edges connected to a specific node.
     *
     * @param key The key of the node.
     * @return A vector of pointers to the edges connected to the node.
     */
    vector<Edge*> getEdges(const string& key);

    /**
     * @brief Removes a specific edge from the graph.
     *
     * @param start_key The key of the start node.
     * @param end_key The key of the end node.
     * @param weight The weight of the edge to remove.
     */
    void removeOneEdge(const string& start_key, const string& end_key, const float& weight);

    /**
     * @brief Removes all outgoing edges from a specific node.
     *
     * @param node A pointer to the node whose outgoing edges should be removed.
     */
    static void removeNodesOutgoingEdges(Node* node);

    /**
     * @brief Removes all incoming edges to a specific node.
     *
     * @param node A pointer to the node whose incoming edges should be removed.
     */
    void removeNodesIncomingEdges(const Node* node);

    /**
     * @brief Removes a node and all its associated edges from the graph.
     *
     * @param key The key of the node to remove.
     */
    void removeNode(const string& key);

    /**
     * @brief Computes the shortest path between two nodes using Dijkstra's algorithm.
     *
     * @param start The key of the starting node.
     * @param end The key of the destination node.
     * @return A vector of pointers to the edges in the shortest path.
     */
    vector<Edge*> dijkstra(const string &start, const string &end);

    /**
     * @brief Sets the visualizer for the graph.
     *
     * @param graphviz A pointer to the GraphVisualizer object.
     */
    void setVisualizer(GraphVisualizer* graphviz) {
        this->graphviz = graphviz;
    }

    /**
     * @brief Gets the current visualizer for the graph.
     *
     * @return A pointer to the GraphVisualizer object.
     */
    [[nodiscard]] GraphVisualizer* getVisualizer() const {
        return graphviz;
    }

private:
    GraphVisualizer* graphviz = nullptr; ///< Pointer to the GraphVisualizer object.
    vector<Node*> nodes; ///< A vector of all nodes in the graph.

    /**
     * @brief Finds a node in the graph by its key.
     *
     * @param key The key of the node to find.
     * @return A pointer to the node, or nullptr if not found.
     */
    Node* findNode(const string &key);
};

#endif // DIGRAPH_H