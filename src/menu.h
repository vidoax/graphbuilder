#ifndef MENU_H
#define MENU_H

#include <iostream>
#include <functional>
#include <limits>
#include <string>

// Forward declarations of DiGraph and DotGraph classes
class DiGraph;
class DotGraph;

/**
 * @class Menu
 * @brief Provides a user interface for interacting with a graph.
 *
 * This class encapsulates functionality for displaying menus, interacting with
 * the user, and performing operations on a graph (e.g., showing nodes, edges,
 * neighbors, saving/loading graphs, etc.).
 */
class Menu {
public:
    /**
     * @brief Displays the main menu and handles user input.
     */
    void showMenu();

    /**
     * @brief Displays all nodes in the graph.
     *
     * @param graph The graph whose nodes are to be displayed.
     */
    static void showNodes(DiGraph &graph);

    /**
     * @brief Displays all edges connected to a specific node.
     *
     * @param graph The graph containing the node.
     * @param key The key (identifier) of the node.
     */
    static void showEdges(DiGraph &graph, const std::string &key);

    /**
     * @brief Displays all neighbors of a specific node.
     *
     * @param graph The graph containing the node.
     * @param key The key (identifier) of the node.
     */
    static void showNeighbors(DiGraph &graph, const std::string &key);

    /**
     * @brief Saves the graph to a file.
     *
     * @param graph The graph to save.
     * @param filename The name of the file to save the graph to.
     */
    static void saveGraphToFile(DiGraph &graph, const std::string& filename);

    /**
     * @brief Loads a graph from a file.
     *
     * @param graph The graph object to populate with data from the file.
     * @param filename The name of the file to load the graph from.
     */
    void loadGraphFromFile(DiGraph &graph, const std::string& filename);

    /**
     * @brief Prompts the user for input and validates it.
     *
     * This template function prompts the user for input of a specific type and
     * validates it using an optional condition. If the input is invalid, an error
     * message is displayed, and the user is prompted again.
     *
     * @tparam T The type of input to expect (e.g., int, std::string, etc.).
     * @param prompt The message to display when prompting for input.
     * @param condition An optional function to validate the input.
     * @param errorMessage The message to display if the input is invalid.
     * @return The validated input value.
     */
    template <typename T>
    T getInput(const std::string& prompt, std::function<bool(T)> condition = nullptr, const std::string& errorMessage = "Invalid input.");

private:
    /**
     * @brief Trims leading and trailing whitespace from a string.
     *
     * @param str The string to trim.
     * @return The trimmed string.
     */
    static std::string trim(const std::string& str);
};

#endif // MENU_H

// Template function implementation
template<typename T>
T Menu::getInput(const std::string &prompt, std::function<bool(T)> condition, const std::string &errorMessage) {
    T value;
    while (true) {
        std::cout << prompt;
        std::cin >> value;

        // Handle invalid input (e.g., non-numeric input for numeric types)
        if (std::cin.fail()) {
            std::cin.clear(); // Clear the error flag
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // Discard invalid input
            std::cout << "Invalid input. Please enter a valid " << typeid(T).name() << "." << std::endl;
            continue;
        }

        // Check custom condition if provided
        if (condition && !condition(value)) {
            std::cout << errorMessage << std::endl;
            continue;
        }

        // Clear the input buffer and return the validated value
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        return value;
    }
}
