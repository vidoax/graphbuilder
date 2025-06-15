#ifndef DOTGRAPH_H
#define DOTGRAPH_H

#include "Node.h"
#include "DiGraph.h"
#include "GraphVisualizer.h"

/**
 * @class DotGraph
 * @brief A concrete implementation of GraphVisualizer for generating DOT format visualizations.
 *
 * This class provides functionality to visualize a graph and its algorithms (e.g., Dijkstra's shortest path)
 * in the DOT format, which can be used with tools like Graphviz to generate graphical representations.
 */
class DotGraph final : public GraphVisualizer {
public:
    /**
     * @brief Visualizes the given graph in DOT format.
     *
     * This method generates a DOT representation of the graph, which can be rendered
     * using tools like Graphviz.
     *
     * @param graph The graph to visualize.
     */
    void visualize(DiGraph& graph) override;

    /**
     * @brief Visualizes Dijkstra's shortest path algorithm on the given graph in DOT format.
     *
     * This method generates a DOT representation of the graph, highlighting the shortest path
     * between the specified start and end nodes.
     *
     * @param graph The graph on which to run Dijkstra's algorithm.
     * @param start The key (identifier) of the starting node.
     * @param end The key (identifier) of the destination node.
     */
    void visualizeDijkstra(DiGraph& graph, const std::string& start, const std::string& end) override;
};

#endif // DOTGRAPH_H
