#include "src/menu.h"

/**
 * @brief Entry point of the program.
 *
 * This function creates an instance of the `Menu` class and calls its `showMenu` method
 * to display the interactive menu for graph operations. The program runs until the user
 * chooses to exit.
 *
 * @return 0 on successful execution.
 */
int main() {
    Menu m; 
    m.showMenu();
    return 0;
}