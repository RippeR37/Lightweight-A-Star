#include "AStar.h"

#include <cmath>
#include <ctime>
#include <vector>
#include <iostream>

int main() {
    srand(static_cast<unsigned int>(time(nullptr)));

    /* Typedefs */
    typedef std::vector<std::vector<int>> MapType; // Map type. AStar instances takes reference to this map!
    typedef std::pair<unsigned int, unsigned int> PositionType; // Position type. In example below its pair of indices (x,y).
    typedef double CostType; // Cost type. Represents type of heuristic and non-heuristic functions cost type.
    
    typedef PathFinder::AStar<MapType, PositionType, CostType> MyAStar; // Shortcut so you don't have to write template arguents all the time


    /** 
     * Map
     * Creation and initialization map.
     */
    MapType map = {
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
    MyAStar::MapAdapter mapAdapter(map); // Adapter for our map to use by AStar instance
    MyAStar::NodeAdapter nodeStart(std::make_pair(1, 1)); // Adapter for start node for AStar instance
    MyAStar::NodeAdapter nodeGoal(std::make_pair(8, 8)); // Adapter for goal node for ASTar istance
    
    // isOnMap(map, position) adapter functionality (checks if position is in bounds of map)
    mapAdapter.isOnMapCallback = [](const MapType& map, const PositionType& position) {
        return (position.first < map.size() && position.second < map[0].size()); // unsigned values measn that there won't be values < 0, no need to check
    };

    // isAvailable(map, position) adapter functionality (checks if algorithm can visit given position)
    mapAdapter.isAvailableCallback = [](const MapType& map, const PositionType& position) {
        return (map[position.second][position.first] == 0);
    };
    
    // Heuristic function - calculates heuristic cost of travel between 'currentNode' and 'goal', i.e. eucilidian distance or manhattan distance
    mapAdapter.heuristicCallback = [](const MapType& map, const MyAStar::NodeAdapter& currentNode, const MyAStar::NodeAdapter& goal) {
        (void) map;

        MyAStar::CostType xCost, yCost;

        xCost = (goal.position.first - currentNode.position.first);
        yCost = (goal.position.second - currentNode.position.second);

        return std::sqrt(xCost*xCost + yCost*yCost);
    };

    // getNeighboursOf(map, node) - returns neighbours of given node (they will be tested with isOnMap and isAvailable functions later)
    mapAdapter.neighboursOf = [&](const MapType& map, const MyAStar::NodeAdapter& node) {
        (void) map;

        std::vector<MyAStar::NodeAdapter> neighbours;

        static const std::vector<std::pair<int, int>> coordDiffs = {
            {  0, -1 }, {  0,  1 }, {  1,  0 }, { -1,  0 }
        };

        for(const std::pair<int, int>& coordDiff : coordDiffs) {
            MyAStar::NodeAdapter neighbour(std::make_pair(node.position.first + coordDiff.first, node.position.second + coordDiff.second));
            neighbours.push_back(neighbour);
        }

        return neighbours;
    };


    /**
     * Querying for shortest path
     */
    MyAStar::Path path = astar.getPath(mapAdapter, nodeStart, nodeGoal);

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

    return 0;
}