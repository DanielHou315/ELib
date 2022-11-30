#ifndef E_PATH
#define E_PATH

#include "eLib/utils/Math.hpp"
using namespace okapi;
using namespace std;

namespace elib{

    struct PathNode{
        Point point;
        QLength targetDistance = 0_m;
        double curvature = 0.0;
        
        PathNode(){
            point.x = 0_m;
            point.y = 0_m;
        }
       PathNode(Point pt, QLength tDist){
           point = pt;
           targetDistance = tDist;
       }
       ~PathNode(){}
    };


    struct PathTurn{
        QAngle targetTurnAngle;
        int nodeIndex;
        PathTurn(){targetTurnAngle = -180_deg;}
        PathTurn(QAngle ang){targetTurnAngle = ang;}
        ~PathTurn(){}
    };

    enum PathDirection {E_PROFILER_PATH_FORWARD, E_PROFILER_PATH_BACKWARD};
    
    struct PathSegment{
        // current direction of the path
        bool direction;
        bool turnAtEnd = false;

        // end point of this path
        int endPointIndex = -1;

        // waypoint
        // x, y -- coordinates of waypoint
        vector<PathNode> nodes;
        
        #if USING_TURN_IN_PURSUIT
        // the turn will be prioritized and the direction of the rest of the path will change. 
        PathTurn turns;
        #endif

        PathSegment(){}
        ~PathSegment(){}

        void append(PathSegment addedPath){
            for(auto pt : addedPath.nodes){nodes.push_back(pt);}
            #if USING_TURN_IN_PURSUIT
            for(auto tu : addedPath.turns){turns.push_back(tu);}
            #endif
        }

        void operator+ (PathSegment anotherPath){
            for(auto pt : anotherPath.nodes){nodes.push_back(pt);}
            #if USING_TURN_IN_PURSUIT
            for(auto tu : anotherPath.turns){turns.push_back(tu);}
            #endif
        }

        int size(){return nodes.size();}
    };


    // Path
    using Path = vector<PathSegment>;
}

#endif