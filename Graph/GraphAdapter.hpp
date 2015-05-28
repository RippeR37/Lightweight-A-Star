#ifndef GRAPH_GRAPHADAPTER_HPP_INCLUDED
#define GRAPH_GRAPHADAPTER_HPP_INCLUDED

#include <vector>
#include <functional>

#include "NodeAdapter.hpp"


namespace Graph {

    template<typename _GraphType, typename _PositionType, typename _CostType = int>
    class GraphAdapter {
        public:
            typedef _GraphType                          GraphType;
            typedef _PositionType                       PositionType;
            typedef _CostType                           CostType;

            typedef NodeAdapter<PositionType, CostType> NodeAdapterType;


        public:
            GraphAdapter(GraphType& graph) : _graph(graph) {
            
            }

            virtual ~GraphAdapter() {
                
            }

            GraphAdapter(const GraphAdapter& graphAdapter) : _graph(graphAdapter._graph) {
                
            }

            GraphAdapter& operator=(const GraphAdapter& graphAdapter) = delete;

            virtual bool isAvailable(const PositionType& position) const = 0;
            virtual std::vector<NodeAdapterType> getNeighboursOf(const NodeAdapterType& node) const = 0;
            
            virtual CostType getHeuristicCostLeft(const NodeAdapterType& currentNode, const NodeAdapterType& goal) const {
                (void) currentNode; // unused here
                (void) goal; // unused here

                return CostType(); // return default value (should be zero-like) for heuristic cost
            }

        protected:
            GraphType& _graph;
    };

}

#endif