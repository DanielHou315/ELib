#ifndef E_BASE_SEARCHER
#define E_BASE_SEARCHER


#include "eLib/eMotionPlanners/Searchers/ESearchNode.hpp"
using namespace elib;
using namespace Eigen;

namespace elib{
    class EBaseSearcher{
    protected:
        // TP Specific
        vector<Vector2i> walls;
        Vector2i worldSize;

        // Cost Function
        virtual uint heuristic(Vector2i pt1, Vector2i pt2) = 0;

        bool isObstacle(Vector2i coordinates_);
        bool isInBound(Vector2i coordinates_);

        // Clear Obstacles
        void clearObstacles();

    public:
        EBaseSearcher(Vector2i wSize_);
        ~EBaseSearcher();

        // Add Obstacle
        void addObstacle(Vector2i coordinates_);

        // Find Path
        virtual vector<Vector2i> findPath(Vector2i source_, Vector2i target_) = 0;
        
    };
}

#endif