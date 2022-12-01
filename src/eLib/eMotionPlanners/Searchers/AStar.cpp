#include "eLib/eMotionPlanners/Searchers/AStar.hpp"
using namespace elib;


namespace elib{

    EASSearcher::EASSearcher(Vector2i worldSize) : EBaseSearcher(worldSize)
    {
        directions = 8;

        Vector2i d1(0, 1);
        Vector2i d2(1, 0);
        Vector2i d3(0, -1);
        Vector2i d4(-1, 0);
        Vector2i d5(-1, -1);
        Vector2i d6(1, 1);
        Vector2i d7(-1, 1);
        Vector2i d8(1, -1);

        direction.push_back(d1);
        direction.push_back(d2);
        direction.push_back(d3);
        direction.push_back(d4);
        direction.push_back(d5);
        direction.push_back(d6);
        direction.push_back(d7);
        direction.push_back(d8);
    }

    // Searcher Node Helper
    SNode* EASSearcher::findNodeOnList(NodeSet& nodes_, Vector2i coordinates_)
    {
        for (auto node : nodes_) {
            if (node->coordinates == coordinates_) {
                return node;
            }
        }
        return nullptr;
    }
    void EASSearcher::releaseNodes(NodeSet& nodes_)
    {
        for (auto it = nodes_.begin(); it != nodes_.end();) {
            delete *it;
            it = nodes_.erase(it);
        }
    }
   
    // Cost Function
    uint EASSearcher::heuristic(Vector2i pt1, Vector2i pt2){
        Vector2d delta;
        delta.x() = pt1.x() - pt2.x();
        delta.y() = pt1.y() - pt2.y();

        // Manhattan
        // return static_cast<uint>(10 * (delta.x() + delta.y()));

        // Euclidean
        return static_cast<uint>(10 * sqrt(pow(delta.x(), 2) + pow(delta.y(), 2)));

        // Octagonal
        // return 10 * (delta.x() + delta.y()) + (-6) * std::min(delta.x(), delta.y());
    }


    void EASSearcher::addMogo(Vector2i coordinates)
    {
        for(int i = floor(coordinates.x()) - MOGO_SAFE_DIST; i < ceil(coordinates.x()) + MOGO_SAFE_DIST;i++){
            for(int j = floor(coordinates.y()) - MOGO_SAFE_DIST; j < ceil(coordinates.y()) + MOGO_SAFE_DIST;j++){
                Vector2i pt(i,j);
                if (isInBound(pt) && hypot(i-coordinates.x(), j-coordinates.y()) <= MOGO_SAFE_DIST){
                    addObstacle(pt);
                }
            }
        }
    }






    vector<Vector2i> EASSearcher::findPath(Vector2i source_, Vector2i target_)
    {
        SNode *current = nullptr;
        NodeSet openSet, closedSet;
        openSet.reserve(800);
        closedSet.reserve(500);
        openSet.push_back(new SNode(source_));

        while (!openSet.empty()) {
            auto current_it = openSet.begin();
            current = *current_it;

            for (auto it = openSet.begin(); it != openSet.end(); it++) {
                auto node = *it;
                if (node->getScore() <= current->getScore()) {
                    current = node;
                    current_it = it;
                }
            }

            if (current->coordinates == target_) {
                break;
            }

            closedSet.push_back(current);
            openSet.erase(current_it);

            for (uint i = 0; i < directions; ++i) {
                Vector2i newCoordinates(current->coordinates + direction[i]);
                if (isObstacle(newCoordinates) ||
                    findNodeOnList(closedSet, newCoordinates)) {
                    continue;
                }

                uint totalCost = current->G + ((i < 4) ? 10 : 14);

                SNode *successor = findNodeOnList(openSet, newCoordinates);
                if (successor == nullptr) {
                    successor = new SNode(newCoordinates, current);
                    successor->G = totalCost;
                    successor->H = heuristic(successor->coordinates, target_);
                    openSet.push_back(successor);
                }
                else if (totalCost < successor->G) {
                    successor->parent = current;
                    successor->G = totalCost;
                }
            }
        }

        vector<Vector2i> path;
        while (current != nullptr) {
            path.push_back(current->coordinates);
            current = current->parent;
        }

        releaseNodes(openSet);
        releaseNodes(closedSet);

        return path;
    }
}