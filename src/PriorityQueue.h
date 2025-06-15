#ifndef PRIORITYQUEUE_H
#define PRIORITYQUEUE_H
#include <string>
#include <utility>
#include <iostream>

//Forward declaration of the PriorityQueue class
template <typename T, typename P> class PriorityQueue;

/**
 *@class Entry
 *@brief Represents an entry in the PriorityQueue.
 *
 *This class stores a node/entry value and prio. Since it's a doubly linked list
 *it also maintains pointer to the next and previous node in the queue.
 *
 *@tparam T type of value stored in entry.
 *@tparam P type of priority associated with value.
 */
template <typename T, typename P>
class Entry {
public:
    // Allow PriorityQueue to access private members.
    friend class PriorityQueue<T, P>;

private:
    /**
     * @brief Constructs a new Entry object.
     *
     * @param prio The priority of the entry.
     * @param value The value stored in the entry.
     */
    Entry(const P prio, T value) :
        _next(nullptr), _prev(nullptr), _value(std::move(value)), _prio(prio) {}

    // Default destructor
    ~Entry() = default;

    // Pointer to the next entry in the queue
    Entry* _next;

    // Pointer to the previous entry in the queue
    Entry* _prev;

    // Value stored in entry
    T _value;

    // Prio of the entry
    P _prio;
};

/**
 * @class PriorityQueue
 * @brief A priority queue implementation using a doubly linked list.
 *
 * This class allows insertion, extraction of the minimum value, and updating
 * the priority of existing values. The queue is ordered by priority, with the
 * smallest priority at the head.
 *
 * @tparam T The type of the values stored in the queue.
 * @tparam P The type of the priorities associated with the values.
 */
template <typename T, typename P>
class PriorityQueue {
public:
    /**
     * @brief Constructs a new Priorityqueue object.
     */
    PriorityQueue() : _head(nullptr), _tail(nullptr) {}

    /**
     * @brief Destroys the PriorityQueue object and frees all allocated memory.
     */
    ~PriorityQueue();

    /**
     * @brief Inserts a new value into the queue with the given priority.
     *
     * If the value already exists, its priority is updated if the new priority
     * is smaller.
     *
     * @param value The value to insert.
     * @param prio The priority of the value.
     */
    void insert(const T &value, const P& prio);

    /**
     * @brief Extracts and removes the value with the smallest priority.
     *
     * @return The value with the smallest priority.
     * @throws std::runtime_error If the queue is empty.
     */
    T extractMin();

    /**
     * @brief Decreases the priority of an existing value.
     *
     * @param value The value whose priority should be decreased.
     * @param prio The new priority (must be smaller than the current priority).
     * @throws std::runtime_error If the value is not found or the new priority
     *         is not smaller than the current priority.
     */

    void decreaseKey(const T &value, P prio);

    /**
     * @brief Removes a value from the queue.
     *
     * @param value The value to remove.
     */

    void remove(const T& value);

    /**
     * @brief Checks if the queue is empty.
     *
     * @return True if the queue is empty, false otherwise.
     */

    [[nodiscard]] bool isEmpty() {
        return _head == nullptr;
    }

    /**
     * @brief Prints the contents of the queue to standard output.
     */

    void printQueue();

private:
    // Pointer to head of the queue
    Entry<T, P>* _head;

    // Pointer to tail of the queue
    Entry<T, P>* _tail;

    /**
     * @brief Checks if a node exists (is not nullptr).
     *
     * @param n The node to check.
     * @return True if the node exists, false otherwise.
     */

    static bool nodeExists(const Entry<T, P>* n) {
        return n != nullptr;
    }

    /**
     * @brief Finds the entry containing the specified value.
     *
     * @param value The value to search for.
     * @return A pointer to the entry containing the value, or nullptr if not found.
     */

    [[nodiscard]] Entry<T, P>* findValue(const T& value);
};

// Destructor
template <typename T, typename P>
PriorityQueue<T, P>::~PriorityQueue() {
    Entry<T, P>* current = _head;
    while (current != nullptr) {
        Entry<T, P>* next = current->_next;
        delete current;
        current = next;
    }
    _head = nullptr;
    _tail = nullptr;
}

// Insert method implementation
template <typename T, typename P>
void PriorityQueue<T, P>::insert(const T &value, const P& prio) {
    // Check if the value already exists
    if (Entry<T, P>* existing = findValue(value)) {
        if (prio < existing->_prio) {
            decreaseKey(value, prio); // Update priority if the new one is smaller
        }
        return;
    }

    // Create a new entry
    auto * new_node = new Entry<T, P>(prio, value);

    // Insert into an empty queue
    if (isEmpty()) {
        _head = new_node;
        _tail = new_node;
        return;
    }

    // Insert at the head if the new priority is the smallest
    if (new_node->_prio < _head->_prio) {
        new_node->_next = _head;
        _head->_prev = new_node;
        _head = new_node;
        return;
    }

    // Insert at the tail if the new priority is the largest
    if (new_node->_prio >= _tail->_prio) {
        new_node->_prev = _tail;
        _tail->_next = new_node;
        _tail = new_node;
        return;
    }

    // Insert in the middle of the queue
    Entry<T, P>* temp = _head->_next;
    while (temp != nullptr) {
        if (new_node->_prio < temp->_prio) {
            new_node->_prev = temp->_prev;
            new_node->_next = temp;
            temp->_prev->_next = new_node;
            temp->_prev = new_node;
            return;
        }
        temp = temp->_next;
    }
}

// ExtractMin method implementation
template<typename T, typename P>
T PriorityQueue<T, P>::extractMin() {
    if (isEmpty()) {
        throw std::runtime_error("Can't extract from an empty queue!");
    }

    T min_value = _head->_value;
    Entry<T, P>* temp_node = _head;
    _head = _head->_next;

    if (_head) {
        _head->_prev = nullptr;
    } else {
        _tail = nullptr; // Queue is now empty
    }

    delete temp_node;
    return min_value;
}

// DecreaseKey method implementation
template <typename T, typename P>
void PriorityQueue<T, P>::decreaseKey(const T &value, P prio) {
    Entry<T, P>* decrease_node = findValue(value);
    if (!decrease_node) {
        throw std::runtime_error("PriorityQueue::decreaseKey: Node not found!");
    }
    if (prio >= decrease_node->_prio) {
        throw std::runtime_error("PriorityQueue::decreaseKey: priority must be smaller than the old one!");
    }

    // Remove and reinsert the node with the new priority
    remove(value);
    insert(value, prio);
}

// Remove method implementation
template <typename T, typename P>
void PriorityQueue<T,P>::remove(const T& value) {
    const Entry<T, P> * remove_node = findValue(value);
    if (!nodeExists(remove_node)) {
        std::cout << "Value wasn't found!" << std::endl;
        return;
    }

    // Handle removal of the only node in the queue
    if (remove_node == _head && remove_node == _tail) {
        _head = _tail = nullptr;
    }

    // Handle removal of the head node
    else if (remove_node == _head) {
        _head = _head->_next;
        _head->_prev = nullptr;
    }

    // Handle removal of the tail node
    else if (remove_node == _tail) {
        _tail = _tail->_prev;
        _tail->_next = nullptr;
    }

    // Handle removal of a middle node
    else {
        remove_node->_prev->_next = remove_node->_next;
        remove_node->_next->_prev = remove_node->_prev;
    }

    delete remove_node;
}

// FindValue method implementation
template <typename T, typename P>
Entry<T, P>* PriorityQueue<T, P>::findValue(const T& value) {
    Entry<T, P> * current = _head;

    while (current != nullptr) {
        if (current->_value == value) {
            return current;
        }
        current = current->_next;
    }
    return nullptr;
}

// PrintQueue method implementation
template <typename T, typename P>
void PriorityQueue<T, P>::printQueue() {
    Entry<T, P> *temp = _head;
    if (isEmpty()) {
        std::cout << "Queue is empty!" << std::endl;
        return;
    }
    while (temp != nullptr) {
        std::cout << temp->_value << " " << temp->_prio << std::endl;
        temp = temp->_next;
    }
}

#endif //PRIORITYQUEUE_H


