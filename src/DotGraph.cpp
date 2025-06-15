#include <iostream>
#include <vector>
#include <unordered_set>
#include <sstream>
#include <iomanip>
#include <algorithm>

#include "DotGraph.h"

/**
 * @brief Generates a DOT representation of the graph.
 *
 * This method outputs the graph in DOT format, which can be rendered using tools like Graphviz.
 * It iterates through all nodes and edges in the graph and prints them in the DOT syntax.
 *
 * @param graph The graph to visualize.
 */
void DotGraph::visualize(DiGraph& graph) {
    // Start the DOT graph definition
    cout << "digraph G {" << endl;

    // Iterate through all nodes in the graph
    for (Node* node : graph.getNodes()) {
        vector<Edge*> edges = node->getEdges();

        // If the node has no edges, print it as a standalone node
        if (edges.empty()) {
            cout << "    \"" << node->getKey() << "\";" << endl;
        }

        // Iterate through all edges of the current node
        for (const Edge* edge : edges) {
            Node* startNode = edge->getStartNode();
            Node* endNode = edge->getEndNode();
            const float weight = edge->getWeight();

            // Print the edge in DOT format with its weight as a label
            if (startNode && endNode) {
                cout << "    \"" << startNode->getKey() << "\" -> \""
                     << endNode->getKey() << "\" [label=\"" << weight << "\"];" << endl;
            }
        }
    }

    // End the DOT graph definition
    cout << "}" << std::endl;
}

/**
 * @brief Generates a DOT representation of the graph with Dijkstra's shortest path highlighted.
 *
 * This method outputs the graph in DOT format, highlighting the shortest path between the
 * specified start and end nodes. Nodes and edges on the shortest path are colored red.
 *
 * @param graph The graph to visualize.
 * @param start The key (identifier) of the starting node.
 * @param end The key (identifier) of the destination node.
 */
void DotGraph::visualizeDijkstra(DiGraph& graph, const std::string& start, const std::string& end) {
    // Get the shortest path using Dijkstra's algorithm
    const auto shortest_path = graph.dijkstra(start, end);
    // Store the edges in the shortest path in a set for quick lookup
    const std::unordered_set<Edge*> shortest_path_set(shortest_path.begin(), shortest_path.end());

    // Create a stringstream to build the DOT output
    std::stringstream output;
    output << "digraph {\n";

    // Get all nodes in the graph
    const auto nodes = graph.getNodes();

    // Define node attributes (color and shape) based on their role in the shortest path
    std::unordered_map<std::string, std::string> node_attributes;
    for (auto* node : nodes) {
        const auto key = node->getKey();
        node_attributes[key] = std::string("node [color=") +
                               (key == start || key == end ? "red" : "black") +
                               ", shape=" +
                               (key == end ? "doublecircle" : "circle") + "]";
    }

    // Calculate the maximum length of node attributes for alignment
    size_t maxAttributeLength = 0;
    for (auto* node : nodes) {
        maxAttributeLength = std::max(maxAttributeLength, node_attributes[node->getKey()].length());
    }

    // Print node definitions with attributes
    for (auto* node : nodes) {
        output << "\t"
               << std::left << std::setw(static_cast<int>(maxAttributeLength))
               << node_attributes[node->getKey()]
               << " \"" << node->getKey() << "\";\n";
    }

    output << "\n";

    // Calculate the maximum length of start and end node names for alignment
    size_t maxStartLength = 0;
    size_t maxEndLength = 0;
    for (auto* node : nodes) {
        for (auto* edge : node->getEdges()) {
            std::string startStr = "\"" + edge->getStartNode()->getKey() + "\"";
            std::string endStr = "\"" + edge->getEndNode()->getKey() + "\"";
            maxStartLength = std::max(maxStartLength, startStr.length());
            maxEndLength = std::max(maxEndLength, endStr.length());
        }
    }

    // Print edge definitions with attributes
    for (auto* node : nodes) {
        for (auto* edge : node->getEdges()) {
            const bool isShortestPath = shortest_path_set.contains(edge);
            std::string startStr = "\"" + edge->getStartNode()->getKey() + "\"";
            std::string endStr = "\"" + edge->getEndNode()->getKey() + "\"";
            output << "\t"
                   << std::left << std::setw(static_cast<int>(maxStartLength)) << startStr
                   << " -> "
                   << std::left << std::setw(static_cast<int>(maxEndLength)) << endStr
                   << " [label=" << edge->getWeight() << ","
                   << " color=" << (isShortestPath ? "red" : "black") << ","
                   << " dir=" << (isShortestPath ? "LR" : "none") << "];\n";
        }
    }

    // End the DOT graph definition
    output << "}\n";
    // Print the DOT output to standard output
    std::cout << output.str();
}