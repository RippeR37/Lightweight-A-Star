#ifndef GRAPH_FUNCTIONALGRAPHADAPTER_HPP_INCLUDED
#define GRAPH_FUNCTIONALGRAPHADAPTER_HPP_INCLUDED

#include <vector>
#include <functional>

#include "GraphAdapter.hpp"


namespace Graph {

    template<typename _GraphType, typename _PositionType, typename _CostType = int>
    class FunctionalGraphAdapter : public GraphAdapter<_GraphType, _PositionType, _CostType> {
        public:
            typedef _GraphType                          GraphType;
            typedef _PositionType                       PositionType;
            typedef _CostType                           CostType;

            typedef NodeAdapter<PositionType, CostType> NodeAdapterType;
            typedef GraphAdapter<_GraphType, _PositionType, _CostType> GraphAdapterBaseType;


        public:
            FunctionalGraphAdapter(GraphType& graph) : GraphAdapterBaseType(graph) {

            }

            FunctionalGraphAdapter(const FunctionalGraphAdapter& graphAdapter) : FunctionalGraphAdapter(graphAdapter._graph) {
                isAvailableCallback = graphAdapter.isAvailableCallback;
                heuristicCallback = graphAdapter.heuristicCallback;
                neighboursOf = graphAdapter.neighboursOf;
            }

            FunctionalGraphAdapter& operator=(const FunctionalGraphAdapter& graphAdapter) = delete;

            bool isAvailable(const PositionType& position) const {
                return isAvailableCallback ? isAvailableCallback(this->_graph, position) : false;
            }

            CostType getHeuristicCostLeft(const NodeAdapterType& currentNode, const NodeAdapterType& goal) const {
                return heuristicCallback ? heuristicCallback(this->_graph, currentNode, goal) : CostType();
            }

            std::vector<NodeAdapterType> getNeighboursOf(const NodeAdapterType& node) const {
                std::vector<NodeAdapterType> result;

                result = neighboursOf(this->_graph, node);

                return result;
            }

            std::function<bool(const GraphType& graph, const PositionType& position)> isAvailableCallback;
            std::function<CostType(const GraphType& graph, const NodeAdapterType& currentNode, const NodeAdapterType& goal)> heuristicCallback;
            std::function<std::vector<NodeAdapterType>(const GraphType& graph, const NodeAdapterType& node)> neighboursOf;
    };

}

#endif
