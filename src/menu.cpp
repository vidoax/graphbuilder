#include "menu.h"
#include <iostream>
#include <fstream>
#include <cctype>
#include <unordered_set>
#include "DotGraph.h"

using namespace std;

/**
 * @brief Displays the main menu and handles user input for graph operations.
 *
 * This method provides a loop for the user to interact with the graph by adding nodes,
 * edges, displaying information, saving/loading graphs, and more.
 */
void Menu::showMenu() {
    int menu_selection;
    bool running = true;
    DiGraph graph;

    while (running) {
        // Display the menu options
        cout << "Graph menu:" << endl;
        cout << "1. add a node" << endl;
        cout << "2. add an edge" << endl;
        cout << "3. show nodes" << endl;
        cout << "4. show edges of a node" << endl;
        cout << "5. show neighbors of node" << endl;
        cout << "6. remove a node" << endl;
        cout << "7. remove an edge" << endl;
        cout << "8. delete the graph" << endl;
        cout << "9. modify an edge" << endl;
        cout << "10. show DotGraph" << endl;
        cout << "11. Shortest path between nodes" << endl;
        cout << "12. Save graph to file" << endl;
        cout << "13. Load graph from file" << endl;
        cout << "14. test driver" << endl;
        cout << "15. exit" << endl;
        cout << endl;

        // Prompt the user for their choice
        cout << "Please enter your choice:";
        cin >> menu_selection;
        cin.clear();
        cin.ignore();

        try {
            // Handle the user's menu selection
            switch (menu_selection) {
                case 1: {
                    // Add a node
                    auto node_name = getInput<string>("Enter the nodes name:");
                    graph.addNode(node_name);
                    cout << "Added node name: " << node_name << endl;
                    cout << endl;
                    break;
                }
                case 2: {
                    // Add an edge
                    auto start_node = getInput<string>("First node name:");
                    auto end_node = getInput<string>("Second node name:");
                    auto edge_weight = getInput<float>("Edge weight:");
                    graph.addEdge(start_node, end_node, edge_weight);
                    cout << start_node << " - " << end_node << " w:" << edge_weight << endl;
                    cout << endl;
                    break;
                }
                case 3: {
                    // Show all nodes in the graph
                    showNodes(graph);
                    break;
                }
                case 4: {
                    // Show edges of a specific node
                    auto key = getInput<string>("Enter name of node to show edges of:");
                    showEdges(graph, key);
                    break;
                }
                case 5: {
                    // Show neighbors of a specific node
                    auto key = getInput<string>("Enter name of node to show neighbors of:");
                    showNeighbors(graph, key);
                    break;
                }
                case 6: {
                    // Remove a node
                    auto key = getInput<string>("Enter name of node to be removed:");
                    graph.removeNode(key);
                    break;
                }
                case 7: {
                    // Remove an edge
                    auto start_key = getInput<string>("Enter the name of the first node:");
                    auto end_key = getInput<string>("Enter the name of the second node:");
                    auto weight = getInput<float>("Enter the weight of the edge:");
                    graph.removeOneEdge(start_key, end_key, weight);
                    cout << start_key << " - " << end_key << " w:" << weight << " REMOVED!" << endl;
                    break;
                }
                case 8: {
                    // Delete the entire graph
                    auto input = getInput<std::string>(
                        "Are you sure to delete the graph? (y/n):",
                        [](std::string in) {
                            std::transform(in.begin(), in.end(), in.begin(), ::tolower);
                            return in == "y" || in == "n" || in == "yes" || in == "no";
                        },
                        "Invalid response. Please enter 'y' or 'n'."
                    );

                    std::transform(input.begin(), input.end(), input.begin(), ::tolower);
                    if (input == "y" || input == "yes") {
                        graph.clear();
                    }
                    break;
                }
                case 9: {
                    // Modify an edge's weight
                    auto start_key = getInput<string>("Enter first node:");
                    auto end_key = getInput<string>("Enter second node:");
                    auto weight = getInput<float>("Enter the weight of the edge:");
                    auto new_weight = getInput<float>("Enter the new weight of the node:");
                    graph.modifyEdgeWeight(start_key, end_key, weight, new_weight);
                    cout << start_key << " - " << end_key << " w:" << new_weight << " ADDED!" << endl;
                    break;
                }
                case 10: {
                    // Visualize the graph using DotGraph
                    DotGraph v;
                    graph.setVisualizer(&v);
                    graph.getVisualizer()->visualize(graph);
                    break;
                }
                case 11: {
                    // Find and visualize the shortest path between two nodes
                    auto start_key = getInput<string>("Enter the first node:");
                    auto end_key = getInput<string>("Enter second node:");
                    DotGraph v;
                    graph.setVisualizer(&v);
                    graph.getVisualizer()->visualizeDijkstra(graph, start_key, end_key);
                    break;
                }
                case 12: {
                    // Save the graph to a file
                    auto filename = getInput<string>("Enter the filename to save graph to:");
                    saveGraphToFile(graph, filename);
                    cout << "Saved graph to " << filename << "!" << endl;
                    break;
                }
                case 13: {
                    // Load a graph from a file
                    auto filename = getInput<string>("Enter the filename to load graph from:");
                    loadGraphFromFile(graph, filename);
                    cout << "Loaded graph from " << filename << "!" << endl;
                    break;
                }
                case 14: {
                    // Test driver: Add predefined nodes and edges
                    graph.addNode("Krefeld");
                    graph.addNode("Duisburg");
                    graph.addNode("Duesseldorf");
                    graph.addNode("Berlin");
                    graph.addNode("Dormagen ander");

                    graph.addEdge("Krefeld", "Duesseldorf", 20.0);
                    graph.addEdge("Krefeld", "Duisburg", 10);
                    graph.addEdge("Duisburg", "Duesseldorf", 15);
                    graph.addEdge("Duisburg", "Berlin", 75);
                    graph.addEdge("Duesseldorf", "Dormagen ander", 45);
                    graph.addEdge("Berlin", "Duesseldorf", 95);
                    graph.addEdge("Berlin", "Dormagen ander", 3);
                    graph.addEdge("Dormagen ander", "Berlin", 7.5);
                    cout << "Added some nodes and edges!" << endl;
                    break;
                }
                case 15: {
                    // Exit the program
                    running = false;
                    break;
                }
                default:
                    throw runtime_error("Invalid input!");
            }
        } catch (runtime_error& e) {
            // Handle runtime errors
            cerr << "ERROR: " << e.what() << endl;
        }
    }
}

/**
 * @brief Displays the neighbors of a specific node.
 *
 * @param graph The graph to search.
 * @param key The key (identifier) of the node.
 */
void Menu::showNeighbors(DiGraph &graph, const string &key) {
    for (vector<Node*> neighbors = graph.getNeighbors(key); const auto neighbor : neighbors) {
        cout << neighbor->getKey() << endl;
    }
}

/**
 * @brief Displays all nodes in the graph.
 *
 * @param graph The graph to display nodes from.
 * @throws runtime_error If no nodes are found.
 */
void Menu::showNodes(DiGraph &graph) {
    vector<Node*> nodes = graph.getNodes();
    if (nodes.empty()) {
        throw runtime_error("No nodes found!");
    }
    for (Node* node : nodes) {
        cout << "-" << node->getKey() << endl;
    }
}

/**
 * @brief Displays all edges connected to a specific node.
 *
 * @param graph The graph to search.
 * @param key The key (identifier) of the node.
 * @throws runtime_error If no edges are found.
 */
void Menu::showEdges(DiGraph &graph, const string &key) {
    const vector<Edge*> node_edges = graph.getEdges(key);

    if (node_edges.empty()) {
        throw runtime_error("No edges found!");
    }
    for (const Edge* edge : node_edges) {
        cout << edge->getStartNode()->getKey() << " - " << edge->getEndNode()->getKey()
             << " w: " << edge->getWeight() << endl;
    }
}

/**
 * @brief Saves the graph to a file.
 *
 * @param graph The graph to save.
 * @param filename The name of the file to save the graph to.
 * @throws runtime_error If the file cannot be opened or created.
 */
void Menu::saveGraphToFile(DiGraph &graph, const string& filename) {
    ofstream file(filename + ".txt");
    if (!file.is_open()) {
        throw runtime_error("Could not open or create file " + filename);
    }
    file << "# All nodes" << endl;

    // Write all nodes to the file
    for (Node* node : graph.getNodes()) {
        file << node->getKey() << endl;
    }

    file << "# All edges" << endl;
    // Write all edges to the file
    for (Node* node : graph.getNodes()) {
        for (const Edge* edge : node->getEdges()) {
            file << edge->getStartNode()->getKey() << " -> "
                 << edge->getEndNode()->getKey() << " | "
                 << edge->getWeight() << endl;
        }
    }

    file << "# end" << endl;
    file.close();
}

/**
 * @brief Loads a graph from a file.
 *
 * @param graph The graph object to populate with data from the file.
 * @param filename The name of the file to load the graph from.
 * @throws runtime_error If the file cannot be opened or contains invalid data.
 */
void Menu::loadGraphFromFile(DiGraph &graph, const string& filename) {
    // Check if the graph is not empty and prompt the user to save it
    if (!graph.getNodes().empty()) {
        cout << "Graph is not empty..." << endl;
        cout << "Would you like to save the current graph?" << endl;

        while (true) {
            const char input = getInput<char>("Enter (y) or (n):");
            if (input == 'y') {
                const auto old_filename = getInput<string>("Enter the filename to save the old graph:");
                saveGraphToFile(graph, old_filename);
                cout << "Saved graph to " << old_filename << "!" << endl;
                break;
            }
            if (input == 'n') {
                break;
            }
        }
    }

    // Clear the current graph and load from the file
    graph.clear();
    ifstream file(filename + ".txt");
    if (!file.is_open()) {
        throw runtime_error("Could not open the file \"" + filename + "\"");
    }

    string line;
    unordered_set<string> node_set;
    bool reading_nodes = false;
    bool reading_edges = false;

    // Read the file line by line
    while (getline(file, line)) {
        line = trim(line);

        if (line.empty()) {
            continue;
        }

        // Handle comments and section headers
        if (line[0] == '#') {
            if (line.find("All nodes") != string::npos) {
                reading_nodes = true;
                reading_edges = false;
                continue;
            }
            if (line.find("All edges") != string::npos) {
                reading_nodes = false;
                reading_edges = true;
                continue;
            }
            if (line.find("end") != string::npos) {
                break;
            }
        }

        // Read nodes
        if (reading_nodes) {
            if (line[0] == '#') {
                continue;
            }

            const string& node = line;
            if (node_set.contains(node)) {
                throw runtime_error("The node " + node + " is listed twice!");
            }
            node_set.insert(node);
            graph.addNode(node);
        }

        // Read edges
        else if (reading_edges) {
            if (line[0] == '#') {
                continue;
            }

            auto arrowPos = line.find("->");
            if (arrowPos == string::npos) {
                throw runtime_error("Invalid edge format (missing '->') in line: " + line);
            }
            auto pipePos = line.find('|', arrowPos + 2);
            if (pipePos == string::npos) {
                throw runtime_error("Invalid edge format (missing '|') in line: " + line);
            }

            string source = trim(line.substr(0, arrowPos));
            string dest = trim(line.substr(arrowPos + 2, pipePos - (arrowPos + 2)));
            string weight_string = trim(line.substr(pipePos + 1));

            // Convert weight to float
            float weight = 0.0f;
            try {
                weight = std::stof(weight_string);
            } catch (const std::exception &) {
                throw std::runtime_error("Invalid weight value in line: " + line);
            }

            // Validate nodes
            if (!node_set.contains(source)) {
                throw runtime_error("The source node " + source + " does not exist in the node set!");
            }
            if (!node_set.contains(dest)) {
                throw runtime_error("The destination node " + dest + " does not exist in the node set!");
            }
            graph.addEdge(source, dest, weight);
        }
    }
}

/**
 * @brief Trims leading and trailing whitespace from a string.
 *
 * @param str The string to trim.
 * @return The trimmed string.
 */
string Menu::trim(const string &str) {
    const size_t start = str.find_first_not_of(" \t\n\r\f\v");
    if (start == string::npos)
        return "";
    const size_t end = str.find_last_not_of(" \t\n\r\f\v");
    return str.substr(start, end - start + 1);
}