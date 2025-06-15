#include "DiGraph.h"

using namespace std;

/**
 * @brief Destroys the DiGraph object and frees allocated memory.
 *
 * This destructor calls the `clear` method to delete all nodes and edges in the graph.
 */
DiGraph::~DiGraph() {
    clear();
}

/**
 * @brief Clears the graph by removing all nodes and edges.
 *
 * This method deletes all nodes and their associated edges, then clears the nodes vector.
 */
void DiGraph::clear() {
    for (Node* node : nodes) {
        for (Edge* edge : node->edges) {
            delete edge; // Delete each edge
        }
        delete node; // Delete each node
    }
    nodes.clear(); // Clear the nodes vector
}

/**
 * @brief Finds a node in the graph by its key.
 *
 * @param key The key of the node to find.
 * @return A pointer to the node, or nullptr if not found.
 */
Node* DiGraph::findNode(const string &key) {
    for (Node *node : nodes) {
        if (node->getKey() == key) {
            return node;
        }
    }
    return nullptr; // Node not found
}

/**
 * @brief Adds a node to the graph.
 *
 * @param key The unique identifier for the node.
 * @throws runtime_error If a node with the same key already exists.
 */
void DiGraph::addNode(const string &key) {
    if (findNode(key) != nullptr) {
        throw runtime_error("Node already in the graph!");
    }

    const auto new_node = new Node(key); // Create a new node
    nodes.push_back(new_node); // Add the node to the graph
}

/**
 * @brief Adds a directed edge between two nodes with a specified weight.
 *
 * @param key1 The key of the start node.
 * @param key2 The key of the end node.
 * @param weight The weight of the edge.
 * @throws runtime_error If either node is not found or the edge already exists.
 */
void DiGraph::addEdge(const string &key1, const string &key2, const float &weight) {
    Node* startNode = findNode(key1);
    if (startNode == nullptr) {
        throw runtime_error("1. Node not found!");
    }

    Node* endNode = findNode(key2);
    if (endNode == nullptr) {
        throw runtime_error("2. Node not found!");
    }

    // Check if the edge already exists
    for (Edge* edge : startNode->edges) {
        if (edge->getEndNode() == endNode && edge->getWeight() == weight) {
            throw runtime_error("Edge already in the graph!");
        }
    }

    Edge* new_edge = new Edge(startNode, endNode, weight); // Create a new edge
    startNode->setEdge(new_edge); // Add the edge to the start node
}

/**
 * @brief Gets all edges connected to a specific node.
 *
 * @param key The key of the node.
 * @return A vector of pointers to the edges connected to the node.
 * @throws runtime_error If the node is not found.
 */
vector<Edge *> DiGraph::getEdges(const string& key) {
    Node* node = findNode(key);
    if (node == nullptr) {
        throw runtime_error("Key not found!");
    }
    return node->getEdges();
}

/**
 * @brief Gets the neighbors of a specific node.
 *
 * This method returns all nodes that are connected to the given node via outgoing or incoming edges.
 *
 * @param key The key of the node.
 * @return A vector of pointers to the neighboring nodes.
 * @throws runtime_error If the node is not found.
 */
vector<Node *> DiGraph::getNeighbors(const string& key) {
    Node* node = findNode(key);
    if (node == nullptr) {
        throw runtime_error("Node not found!");
    }

    vector<Node *> neighbors;
    vector<Edge*> node_edges = node->getEdges();

    // Add outgoing neighbors
    for (Edge* edge : node_edges) {
        Node* neighbor = edge->getEndNode();
        neighbors.push_back(neighbor);
    }

    // Add incoming neighbors
    for (Node* n : nodes) {
        for (Edge* edge : n->getEdges()) {
            if (edge->getEndNode() == node) {
                Node* neighbor = edge->getStartNode();
                if (ranges::find(neighbors, neighbor) == neighbors.end()) {
                    neighbors.push_back(neighbor);
                }
            }
        }
    }

    return neighbors;
}

/**
 * @brief Removes a node and all its associated edges from the graph.
 *
 * @param key The key of the node to remove.
 * @throws runtime_error If the node is not found.
 */
void DiGraph::removeNode(const string& key) {
    Node* node = findNode(key);
    if (!node) {
        throw runtime_error("Key not found!");
    }

    // Remove outgoing and incoming edges
    removeNodesOutgoingEdges(node);
    removeNodesIncomingEdges(node);

    // Remove the node from the graph
    if (auto it = std::remove(nodes.begin(), nodes.end(), node); it != nodes.end()) {
        delete node; // Free the node's memory
        nodes.erase(it, nodes.end()); // Remove the node from the nodes vector
    }
}

/**
 * @brief Removes a specific edge from the graph.
 *
 * @param start_key The key of the start node.
 * @param end_key The key of the end node.
 * @param weight The weight of the edge to remove.
 * @throws runtime_error If either node or the edge is not found.
 */
void DiGraph::removeOneEdge(const string& start_key, const string& end_key, const float& weight) {
    Node* start_node = findNode(start_key);
    if (start_node == nullptr) {
        throw runtime_error("First node not found!");
    }

    Node* end_node = findNode(end_key);
    if (end_node == nullptr) {
        throw runtime_error("End node not found!");
    }

    vector<Edge*>& node_edges = start_node->getEdges();

    // Find and remove the edge
    for (auto it = node_edges.begin(); it != node_edges.end(); ++it) {
        if ((*it)->getEndNode() == end_node && (*it)->getWeight() == weight) {
            delete *it; // Free the edge's memory
            node_edges.erase(it); // Remove the edge from the node's edge list
            return;
        }
    }
    throw runtime_error("Edge not found!");
}

/**
 * @brief Removes all outgoing edges from a specific node.
 *
 * @param node A pointer to the node whose outgoing edges should be removed.
 */
void DiGraph::removeNodesOutgoingEdges(Node* node) {
    vector<Edge*> node_edges = node->getEdges();
    for (Edge* edge : node_edges) {
        delete edge; // Free each edge's memory
    }
    node_edges.clear(); // Clear the node's edge list
}

/**
 * @brief Removes all incoming edges to a specific node.
 *
 * @param node A pointer to the node whose incoming edges should be removed.
 */
void DiGraph::removeNodesIncomingEdges(const Node* node) {
    for (Node* n : nodes) {
        for (auto it = n->edges.begin(); it != n->edges.end();) {
            if ((*it)->getEndNode() == node) {
                delete *it; // Free the edge's memory
                it = n->edges.erase(it); // Remove the edge from the node's edge list
            } else {
                ++it;
            }
        }
    }
}

/**
 * @brief Modifies the weight of an existing edge.
 *
 * @param key1 The key of the start node.
 * @param key2 The key of the end node.
 * @param old_weight The current weight of the edge.
 * @param new_weight The new weight to assign to the edge.
 * @throws runtime_error If either node or the edge is not found.
 */
void DiGraph::modifyEdgeWeight(const string& key1, const string &key2, const float old_weight, const float new_weight) {
    Node* start_node = findNode(key1);
    if (start_node == nullptr) {
        throw runtime_error("First node not found!");
    }

    const Node* end_node = findNode(key2);
    if (end_node == nullptr) {
        throw runtime_error("End node not found!");
    }

    // Find and update the edge's weight
    for (Edge* edge : start_node->getEdges()) {
        if (edge->getEndNode() == end_node && edge->getWeight() == old_weight) {
            edge->setWeight(new_weight);
            return;
        }
    }
    throw runtime_error("Edge not found!");
}

/**
 * @brief Computes the shortest path between two nodes using Dijkstra's algorithm.
 *
 * @param start The key of the starting node.
 * @param end The key of the destination node.
 * @return A vector of pointers to the edges in the shortest path.
 * @throws runtime_error If either node is not found or no path exists.
 */
vector<Edge*> DiGraph::dijkstra(const string &start, const string &end) {
    unordered_map<Node*, float> distances; // Stores the shortest distance to each node
    unordered_map<Node*, Edge*> previous; // Stores the previous edge in the shortest path
    PriorityQueue<Node*, float> pq; // Priority queue for Dijkstra's algorithm

    Node* startNode = findNode(start);
    Node* endNode = findNode(end);
    if (!startNode || !endNode) {
        throw runtime_error("Start or end node not found!");
    }

    // Initialize distances to infinity
    for (Node* node : nodes) {
        distances[node] = numeric_limits<float>::infinity();
    }

    distances[startNode] = 0; // Distance to the start node is 0
    pq.insert(startNode, 0); // Add the start node to the priority queue

    while (!pq.isEmpty()) {
        Node* current = pq.extractMin(); // Get the node with the smallest distance

        if (current == endNode) break; // Stop if we reach the destination node

        bool hasEdges = false;
        for (Edge* edge : current->getEdges()) {
            hasEdges = true;
            Node* neighbor = edge->getEndNode();

            // Relax the edge
            if (float newDist = distances[current] + edge->getWeight(); newDist < distances[neighbor]) {
                distances[neighbor] = newDist;
                previous[neighbor] = edge;
                pq.insert(neighbor, newDist); // Update the priority queue
            }
        }

        if (current == startNode && !hasEdges) {
            throw runtime_error("Start node has no outgoing edges!");
        }
    }

    // Reconstruct the shortest path
    vector<Edge*> path;
    for (Node* at = endNode; previous.contains(at); at = previous[at]->getStartNode()) {
        path.push_back(previous[at]);
    }

    if (path.empty()) {
        throw runtime_error("No path found from " + start + " to " + end);
    }

    reverse(path.begin(), path.end()); // Reverse the path to get the correct order
    return path;
}








