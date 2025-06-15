#ifndef NODE_H
#define NODE_H

#include <utility>
#include <vector>
#include <string>
#include "Edge.h"

/**
 * @class Node
 * @brief Represents a node in a graph.
 *
 * Each node stores a unique key (identifier) and a list of edges connected to it.
 * The edges represent connections to other nodes in the graph.
 */
class Node {
public:
    /**
     * @brief Constructs a new Node object with the given key.
     *
     * @param key The unique identifier for the node.
     */
    explicit Node(std::string key) : key(std::move(key)) {}

    /**
     * @brief Copy constructor for the Node class.
     *
     * @param old The Node object to copy.
     */
    Node(const Node& old) {
        key = old.key;
    }

    /**
     * @brief Destroys the Node object.
     *
     * The default destructor is used since no dynamic memory is managed directly.
     */
    ~Node() = default;

    /**
     * @brief Gets the key (identifier) of the node.
     *
     * @return The key of the node.
     */
    std::string getKey() { return key; }

    /**
     * @brief Sets the key (identifier) of the node.
     *
     * @param k The new key to assign to the node.
     */
    void setKey(const std::string &k) { key = k; }

    /**
     * @brief Gets the list of edges connected to the node.
     *
     * @return A reference to the vector of edges.
     */
    std::vector<Edge*>& getEdges() { return edges; }

    /**
     * @brief Adds an edge to the node's list of edges.
     *
     * @param edge A pointer to the Edge object to add.
     */
    void setEdge(Edge* edge) { edges.push_back(edge); }

    // Friend declaration to allow DiGraph access to private members
    friend class DiGraph;

private:
    std::string key; // The unique identifier for the node.
    std::vector<Edge*> edges; // A list of edges connected to the node.
};

#endif // NODE_H
