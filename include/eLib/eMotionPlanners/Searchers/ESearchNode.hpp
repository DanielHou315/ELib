#ifndef E_SEARCH_NODE
#define E_SEARCH_NODE

#include "eLib/utils/EMath.hpp"
using namespace Eigen;

namespace elib{

    using uint = unsigned int;

    struct SNode
    {
        uint G, H;
        Vector2i coordinates;
        SNode *parent;

        SNode(Vector2i coord_, SNode *parent_ = nullptr){
            parent = parent_;
            coordinates = coord_;
            G = H = 0;
        }
        uint getScore(){return G + H;}
    };

    using NodeSet = std::vector<SNode*>;
}

#endif