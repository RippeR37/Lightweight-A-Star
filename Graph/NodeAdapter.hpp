#ifndef GRAPH_NODEADAPTER_HPP_INCLUDED
#define GRAPH_NODEADAPTER_HPP_INCLUDED


namespace Graph {

    template<typename _PositionType, typename _CostType = int>
    class NodeAdapter {
        public:
            typedef _PositionType                   PositionType;
            typedef _CostType                       CostType;


        public:
            NodeAdapter(const PositionType& positionArg) : position(positionArg) {
                _costF = CostType();
                _costG = CostType();
                _costH = CostType();
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

}

#endif
