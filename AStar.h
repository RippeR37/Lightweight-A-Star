#ifndef ASTAR_H_INCLUDED
#define ASTAR_H_INCLUDED

#include <map>
#include <set>
#include <list>
#include <vector>
#include <functional>
#include <algorithm>

namespace PathFinder {

    template<typename _MapType, typename _PositionType, typename _CostType = int>
    class AStar {
        /**
         * Type definitions
         */
        public:
            typedef _MapType                        MapType;
            typedef _PositionType                   PositionType;
            typedef _CostType                       CostType;
            typedef std::list<PositionType>         Path;

            class MapAdapter;
            class NodeAdapter;


        /**
         * MapAdapter API
         */
        public:
            class MapAdapter {
                public:
                    MapAdapter(MapType& map) : _map(map) { }
                    MapAdapter(const MapAdapter& mapAdapter) : _map(mapAdapter._map) {
                        isOnMapCallback = mapAdapter.isOnMapCallback;
                        isAvailableCallback = mapAdapter.isAvailableCallback;
                        heuristicCallback = mapAdapter.heuristicCallback;
                        neighboursOf = mapAdapter.neighboursOf;
                    }

                    MapAdapter& operator=(const MapAdapter& mapAdapter) = delete;

                    bool isOnMap(const PositionType& position) const {
                        return isOnMapCallback ? isOnMapCallback(_map, position) : false;
                    }

                    bool isAvailable(const PositionType& position) const {
                        return isAvailableCallback ? isAvailableCallback(_map, position) : false;
                    }

                    CostType getHeuristicCostLeft(const NodeAdapter& currentNode, const NodeAdapter& goal) const {
                        return heuristicCallback ? heuristicCallback(_map, currentNode, goal) : CostType();
                    }

                    std::vector<NodeAdapter> getNeighboursOf(const NodeAdapter& node) const {
                        std::vector<NodeAdapter> result;

                        if(neighboursOf) {
                            result = neighboursOf(_map, node);
                            std::random_shuffle(result.begin(), result.end());
                        }

                        return result;
                    }

                    std::function<bool(const MapType& map, const PositionType& position)> isOnMapCallback;
                    std::function<bool(const MapType& map, const PositionType& position)> isAvailableCallback;
                    std::function<CostType(const MapType& map, const NodeAdapter& currentNode, const NodeAdapter& goal)> heuristicCallback;
                    std::function<std::vector<NodeAdapter>(const MapType& map, const NodeAdapter& node)> neighboursOf;

                private:
                    MapType& _map;
            };


        /**
         * NodeAdapter
         */
        public:
            class NodeAdapter {
                public:
                    NodeAdapter(const PositionType& positionArg) : position(positionArg) {

                    }

                    NodeAdapter(const NodeAdapter& node) : position(node.position) {
                        _costF = node._costF;
                        _costG = node._costG;
                        _costH = node._costH;
                    }

                    NodeAdapter& operator=(const NodeAdapter& node) {
                        const_cast<PositionType&>(position) = node.position;

                        _costF = node._costF;
                        _costG = node._costG;
                        _costH = node._costH;

                        return *this;
                    }

                    bool operator==(const NodeAdapter& node) const {
                        return position == node.position;
                    }

                    bool operator<(const NodeAdapter& node) const {
                        return (f() < node.f() || (f() == node.f() && (h() < node.h() || (h() == node.h() && (position < node.position)))));
                    }

                    CostType f() const { return _costF; }
                    CostType g() const { return _costG; }
                    CostType h() const { return _costH; }

                    void g(CostType value) {
                        _costG = value;
                        _updateCostF();
                    }

                    void h(CostType value) {
                        _costH = value;
                        _updateCostF();
                    }

                    const PositionType position;

                private:
                    void _updateCostF() {
                        _costF = _costG + _costH;
                    }

                    CostType _costF;
                    CostType _costG;
                    CostType _costH;
            };


        /**
         * A-Star API
         */
        public: // methods
            AStar() {

            };

            Path getPath(const MapAdapter& mapAdapter, const NodeAdapter& start, const NodeAdapter& goal) const {
                Path resultPath;

                struct NodePositionComparator {
                    bool operator() (const NodeAdapter& lhs, const NodeAdapter& rhs) {
                        return lhs.position < rhs.position;
                    }
                };

                std::set<NodeAdapter> open = { start };
                std::set<NodeAdapter, NodePositionComparator> closed;
                std::map<NodeAdapter, NodeAdapter, NodePositionComparator> came_from;
                typename std::set<NodeAdapter>::iterator current;

                while(open.empty() == false) {
                    current = open.begin();

                    if(current->position == goal.position) {
                        // Goal found, recreating path from 'goal' node to 'start' node
                        auto pathCurrent = came_from.find(*current);
                        if(pathCurrent != came_from.end()) {
                            while(pathCurrent->second.position != start.position) {
                                resultPath.push_front(pathCurrent->second.position);
                                pathCurrent = came_from.find(pathCurrent->second);
                            }
                        }

                        return resultPath;
                    }

                    closed.insert(*current);

                    std::vector<NodeAdapter> neighbours = mapAdapter.getNeighboursOf(*current);
                    for(NodeAdapter& neighbour : neighbours) {
                        if(mapAdapter.isOnMap(neighbour.position) && mapAdapter.isAvailable(neighbour.position)) {
                            typename std::set<NodeAdapter>::iterator cIter, oIter;

                            cIter = closed.find(neighbour);
                            if(cIter != closed.end()) {
                                continue;
                            }

                            CostType hypotheticalNeighbourCostG = current->g() + 1;

                            oIter = open.find(neighbour);
                            if(oIter == open.end() || hypotheticalNeighbourCostG < oIter->g()) {
                                if(oIter != open.end()) {
                                    open.erase(oIter);
                                }

                                auto cameFromIter = came_from.find(neighbour);
                                if(cameFromIter != came_from.end())
                                    came_from.erase(cameFromIter);
                                came_from.emplace(std::pair<NodeAdapter, NodeAdapter>(neighbour, *current));

                                neighbour.g(hypotheticalNeighbourCostG);
                                neighbour.h(mapAdapter.getHeuristicCostLeft(neighbour, goal));
                                open.insert(neighbour);
                            }
                        }
                    }

                    open.erase(current);
                }

                // If algorithm comes here, no path was found, and return value of this method will be empty vector

                return resultPath;
            }
    };

}

#endif // ASTAR_H_INCLUDED
