#ifndef E_ASTAR
#define E_ASTAR

#include <vector>
#include <functional>
#include <set>
using namespace std;

#include "eLib/eMotionPlanners/Searchers/EBaseSearcher.hpp"
using namespace elib;
using namespace Eigen;
using namespace okapi;

namespace elib
{

    #define MOGO_SAFE_DIST 5.0

    class EASSearcher : public EBaseSearcher
    {
    private:
        // TP Specific
        vector<Vector2i> direction;
        uint directions;
        
        // Cost Function
        virtual uint heuristic(Vector2i pt1, Vector2i pt2);

        SNode* findNodeOnList(NodeSet& nodes_, Vector2i coordinates_);
        void releaseNodes(NodeSet& nodes_);

    public:
        EASSearcher(Vector2i worldSize);

        // Add Obstacle
        void addMogo(Vector2i coordinates);

        // Find Path
        virtual vector<Vector2i> findPath(Vector2i source_, Vector2i target_);
    };
}

#endif