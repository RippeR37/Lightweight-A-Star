#include "Graph/AStar.hpp"
#include "Graph/FunctionalGraphAdapter.hpp"

#include <cmath>
#include <vector>
#include <iostream>

/**
 * Example of use of AStar class implementint A* pathfind algorithm.
 *
 * This example uses Graph::FunctionalGraphAdapter template which allows programmer to bind custom functions
 * instead of creating your own class inheriting from Graph::GraphAdapter and implementing it's API there.
 */


/* Typedefs */
typedef std::vector<std::vector<int>> GraphType; // Graph type. AStar instances takes reference to this graph!
typedef std::pair<unsigned int, unsigned int> PositionType; // Position type. In example below its pair of indices (x, y).
typedef double CostType; // Cost type. Represents type of heuristic and non-heuristic functions cost type.

typedef Graph::AStar<GraphType, PositionType, CostType> MyAStar; // Shortcut so you don't have to write template arguents all the time
typedef Graph::FunctionalGraphAdapter<GraphType, PositionType, CostType> MyGraphAdapter; // Type for our graph adapter for which you can bind functions


int main() {
    /**
     * Map
     * Creation and initialization map.
     */
    GraphType map = {
        { 0, 0, 0, 0, 0, 0, 0, 1, 0, 0 },
        { 0, 0, 0, 0, 0, 1, 0, 1, 0, 0 },
        { 0, 0, 0, 0, 0, 1, 0, 1, 0, 0 },
        { 0, 0, 0, 0, 0, 1, 0, 0, 0, 0 },
        { 0, 0, 0, 0, 0, 1, 1, 1, 1, 0 },
        { 0, 0, 0, 0, 0, 1, 0, 0, 0, 0 },
        { 0, 0, 0, 0, 0, 1, 0, 0, 0, 0 },
        { 0, 0, 1, 1, 1, 1, 0, 0, 0, 0 },
        { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
        { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
    };


    /**
     * Code
     * Creation and initialization of AStar instance, mapAdapter for AStar and 'start' and 'goal' nodes.
     * Providing adapter functionality for MapAdapter.
     */
    MyAStar astar; // Instance of AStar
    MyGraphAdapter graphAdapter(map); // Adapter for our map to use by AStar instance. This type have binded functions instead of inheritance
    MyAStar::NodeAdapterType nodeStart(std::make_pair(1, 1)); // Adapter for start node for AStar instance
    MyAStar::NodeAdapterType nodeGoal(std::make_pair(8, 8)); // Adapter for goal node for ASTar istance

    // isAvailable(graph, position) adapter functionality (checks if algorithm can visit node with given position)
    graphAdapter.isAvailableCallback = [](const GraphType& graph, const PositionType& position) {
        bool isOnMap;
        bool isAvailable = false;

        // check if positions is 'on' our map
        isOnMap = (position.first < graph.size() && position.second < graph[0].size());

        // check if it's value is zero
        if(isOnMap)
            isAvailable = (graph[position.second][position.first] == 0);

        return isAvailable;
    };

    // Heuristic function - calculates heuristic cost of travel between 'currentNode' and 'goal', i.e. eucilidian distance or manhattan distance
    graphAdapter.heuristicCallback = [](const GraphType& graph, const MyAStar::NodeAdapterType& currentNode, const MyAStar::NodeAdapterType& goal) {
        MyAStar::CostType xCost, yCost;
        (void) graph; // unused here

        xCost = (goal.position.first - currentNode.position.first);
        yCost = (goal.position.second - currentNode.position.second);

        return std::sqrt(xCost*xCost + yCost*yCost);
    };

    // getNeighboursOf(graph, node) - returns neighbours of given node (they will be tested with and isAvailable function later)
    graphAdapter.neighboursOf = [&](const GraphType& graph, const MyAStar::NodeAdapterType& node) {
        (void) graph; // unused here

        std::vector<MyAStar::NodeAdapterType> neighbours;

        static const std::vector<std::pair<int, int>> coordDiffs = {
            {  0, -1 }, {  0,  1 }, {  1,  0 }, { -1,  0 }
        };

        for(const std::pair<int, int>& coordDiff : coordDiffs) {
            MyAStar::NodeAdapterType neighbour(std::make_pair(node.position.first + coordDiff.first, node.position.second + coordDiff.second));
            neighbours.push_back(neighbour);
        }

        return neighbours;
    };


    /**
     * Querying for shortest path
     */
    MyAStar::PathType path = astar.getPath(graphAdapter, nodeStart, nodeGoal);

    // Marks path on our map
    for(PositionType& pos : path) {
        map[pos.second][pos.first] = '.';
    }

    // Prints out map
    map[nodeStart.position.second][nodeStart.position.first] = 'S';
    map[nodeGoal.position.second][nodeGoal.position.first] = 'E';

    for(unsigned int i = 0; i < map.size(); ++i) {
        for(unsigned int j = 0; j < map[i].size(); ++j) {
            std::cout << (char)map[i][j];
        }
        std::cout << std::endl;
    }

    system("pause");

    return 0;
}
