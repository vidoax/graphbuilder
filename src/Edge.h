#ifndef EDGE_H
#define EDGE_H

// Forward declaration of the Node class
class Node;

/**
 * @class Edge
 * @brief Represents an edge in a graph.
 *
 * An edge connects two nodes (start and end) and has an associated weight.
 * This class provides methods to access and modify the edge's properties.
 */
class Edge {
public:
    /**
     * @brief Constructs a new Edge object.
     *
     * @param start_n A pointer to the start node of the edge.
     * @param end_n A pointer to the end node of the edge.
     * @param w The weight of the edge.
     */
    explicit Edge(Node* start_n, Node* end_n, float w)
        : _start_node(start_n), _end_node(end_n), _weight(w) {}

    /**
     * @brief Gets the weight of the edge.
     *
     * @return The weight of the edge.
     */
    [[nodiscard]] float getWeight() const { return _weight; }

    /**
     * @brief Sets the weight of the edge.
     *
     * @param w The new weight to assign to the edge.
     */
    void setWeight(float w) { _weight = w; }

    /**
     * @brief Gets the start node of the edge.
     *
     * @return A pointer to the start node.
     */
    [[nodiscard]] Node* getStartNode() const { return _start_node; }

    /**
     * @brief Sets the start node of the edge.
     *
     * @param n A pointer to the new start node.
     */
    void setStartNode(Node* n) { _start_node = n; }

    /**
     * @brief Gets the end node of the edge.
     *
     * @return A pointer to the end node.
     */
    [[nodiscard]] Node* getEndNode() const { return _end_node; }

    /**
     * @brief Sets the end node of the edge.
     *
     * @param n A pointer to the new end node.
     */
    void setEndNode(Node* n) { _end_node = n; }

    // Friend declaration to allow DiGraph access to private members
    friend class DiGraph;

private:
    Node* _start_node; // Pointer to the start node of the edge.
    Node* _end_node;   // Pointer to the end node of the edge.
    float _weight;     // The weight of the edge.
};

#endif // EDGE_H
