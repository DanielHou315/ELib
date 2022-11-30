#include "eLib/eMotionPlanners/Searchers/EBaseSearcher.hpp"
using namespace elib;
using namespace Eigen;

namespace elib{
    // -----------------------------------
    //
    // Base Searcher
    // 
    // -----------------------------------
    // Costructor
    EBaseSearcher::EBaseSearcher(Vector2i wSize_){worldSize = wSize_;}
    EBaseSearcher::~EBaseSearcher(){}

    // Obstacles
    void EBaseSearcher::addObstacle(Vector2i coordinates_){walls.push_back(coordinates_);}
    // Detect if point is obstacle
    bool EBaseSearcher::isObstacle(Vector2i coordinates_){
        if (coordinates_.x() < 0 || coordinates_.x() >= worldSize.x()
            || coordinates_.y() < 0 || coordinates_.y() >= worldSize.y()
            || std::find(walls.begin(), walls.end(), coordinates_) != walls.end())
            {return true;}
        return false;
    }
    bool EBaseSearcher::isInBound(Vector2i coordinates_){
        return (coordinates_.x() >= 0 && coordinates_.x() < worldSize.x() &&
            coordinates_.y() >= 0 && coordinates_.y() < worldSize.y());
    }

    // Clear Obstacles
    void EBaseSearcher::clearObstacles(){walls.clear();}
}