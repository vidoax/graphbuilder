#ifndef GRAPHVISUALIZER_H
#define GRAPHVISUALIZER_H

// Forward declaration of the DiGraph class
class DiGraph;

/**
 * @class GraphVisualizer
 * @brief An abstract base class for visualizing graphs and graph algorithms.
 *
 * This class defines an interface for visualizing a graph and its algorithms,
 * such as Dijkstra's shortest path algorithm. Concrete implementations of this
 * class can provide specific visualization logic (e.g., using a graphical library
 * or generating DOT files).
 */
class GraphVisualizer {
public:
    /**
     * @brief Visualizes the given graph.
     *
     * This pure virtual function must be implemented by derived classes to provide
     * specific visualization logic for the graph.
     *
     * @param graph The graph to visualize.
     */
    virtual void visualize(DiGraph& graph) = 0;

    /**
     * @brief Visualizes Dijkstra's shortest path algorithm on the given graph.
     *
     * This pure virtual function must be implemented by derived classes to provide
     * specific visualization logic for Dijkstra's algorithm.
     *
     * @param graph The graph on which to run Dijkstra's algorithm.
     * @param start The key (identifier) of the starting node.
     * @param end The key (identifier) of the destination node.
     */
    virtual void visualizeDijkstra(DiGraph& graph, const std::string& start, const std::string& end) = 0;

    /**
     * @brief Virtual destructor for proper cleanup in derived classes.
     *
     * Ensures that derived classes can safely override this destructor if needed.
     */
    virtual ~GraphVisualizer() = default;
};

#endif // GRAPHVISUALIZER_H
