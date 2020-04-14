#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    start_node = &model.FindClosestNode(start_x, start_y);
    //std::cout << "start node x: " << start_node->x << ", y: " << start_node->y << "\n";

    end_node = &model.FindClosestNode(end_x, end_y);
    //std::cout << "end node x: " << end_node->x << ", y: " << end_node->y << "\n";
}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();

    for (RouteModel::Node *neighbor : current_node->neighbors) {
        if (!neighbor->visited) {
            neighbor->visited = true;
            neighbor->parent = current_node;
            /*
             * Remember that the g-value of the new neighbor has to correspond to the path
             * starting from the start node going through the current node. Hence the new
             * g-value is calculated as:
             * g-value of current node + distance of new neighbor from the current node
             * 
             * This is different from:
             * distance of new value from the start node
             */
            neighbor->g_value = current_node->g_value + neighbor->distance(*current_node);
            neighbor->h_value = CalculateHValue(neighbor);

            open_list.push_back(neighbor);
            //std::cout << "pushed neighbor, x: " << neighcurrent_nodebor->x << ", y: " << neighbor->y 
            //<< ", g: " << neighbor->g_value << ", h: " << neighbor->h_value << "\n";
        }
    }
}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

RouteModel::Node *RoutePlanner::NextNode() {
    /* return null if the open list has no more nodes */
    if (open_list.empty()) {
        return nullptr;
    }

    /* sort the open list base on g+h value of nodes */
    sort(open_list.begin(), open_list.end(), Compare);
    RouteModel::Node* ret = open_list.back();
    open_list.pop_back();

    //std::cout << "Returning node with x: " << ret->x << ", y: " << ret->y << "\n";
    return ret;
}

/*
 * Helper function to sort the list of vectors in descending order based on (g+h) value.
 */
bool RoutePlanner::Compare(RouteModel::Node* node1,
            RouteModel::Node* node2) {
    return (node1->g_value + node1->h_value) > (node2->g_value + node2->h_value);
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here.
    /*
     * Terminate the loop when the current node becomes null. This would indicate that the
     * start node was reached since its parent pointer was initialized to null in
     * AStarSearch() method.
     */
    while (current_node != nullptr) {
        path_found.push_back(*current_node);
        if (current_node->parent != nullptr) {
            distance = distance + current_node->distance(*(current_node->parent));
        }
        current_node = current_node->parent;
    }

    reverse(path_found.begin(), path_found.end());
    //std::cout << "path size: " << path_found.size() << std::endl;
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    //std::cout << "path distance: " << distance << std::endl;
    return path_found;
}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // TODO: Implement your solution here.
    //open_list.push_back(current_node);
    start_node->visited = true;
    start_node->parent = nullptr;
    current_node = start_node;

    while (current_node != end_node && current_node != nullptr) {
        AddNeighbors(current_node);
        current_node = NextNode();
    }

    /* Handle the case where the open_list becomes empty before reaching the end node */
    if (current_node == nullptr) {
        //std::cout << "Current node is null. Path not found, returning\n";
        return;
    }

    /* If the current node was not null, then a path was not found */
    m_Model.path = (current_node == end_node) ? ConstructFinalPath(current_node) : ConstructFinalPath(nullptr);
}