#include "eLib/eMotionPlanners/Profilers/LinearInterpolater.hpp"

namespace elib{
    // ----------------------------------------------------------
    // 
    // Linear Interpolater
    // 
    // ----------------------------------------------------------
    ELinearInterpolater::ELinearInterpolater() : EBaseInterpolater() {}
    ELinearInterpolater::~ELinearInterpolater(){}

    PathSegment ELinearInterpolater::interpolatePath(vector<Point> &pointSet, bool direction){
        PathSegment path;
        path.direction = direction;
        PathNode node;

        QLength dx, dy, dl;

        QLength unit_vec_x, unit_vec_y;

        // For each Pair of Points
        for(auto i = 0; i < pointSet.size()-1;i++){
            dx = pointSet[i+1].x - pointSet[i].x;
            dy = pointSet[i+1].y - pointSet[i].y;

            dl = sqrt(dx*dx+dy*dy);
            unit_vec_x = dx / dl.convert(inch);
            unit_vec_y = dy / dl.convert(inch);

            int pointNum = ceil(dl.convert(inch));
            // Insert Points
            for (int j = 0; j < ceil(pointNum); j++){
                node.point.x = pointSet[i].x + unit_vec_x*j;
                node.point.y = pointSet[i].y + unit_vec_y*j;
                path.nodes.push_back(node);
            }
            // If this is the end, push the last point in there
            if (i == pointSet.size()-2){
                node.point.x = pointSet[i+1].x;
                node.point.y = pointSet[i+1].y;
                path.nodes.push_back(node);
                path.endPointIndex = path.nodes.size()-1;
            }
        }

        // Compute Target Distance
        calcTargetDistance(path, direction);
        
        // Compute Curvature
        // TBD

        // Add an end path. 
        addEndSegment(path, unit_vec_x, unit_vec_y, direction);

        // Return Path
        return path;
    }

    PathSegment ELinearInterpolater::interpolatePath(Point &sPoint, Point &ePoint, bool direction){
        PathSegment path;
        path.direction = direction;
        PathNode node;

        QLength dx = ePoint.x - sPoint.x;
        QLength dy = ePoint.y - sPoint.y;

        QLength dl = sqrt(dx*dx+dy*dy);
        QLength unit_vec_x = dx / dl.convert(inch);
        QLength unit_vec_y = dy / dl.convert(inch);

        int pointNum = ceil(dl.convert(inch));

        // Insert Points
        for (int j = 0; j < ceil(pointNum); j++){
            node.point.x = sPoint.x + unit_vec_x*j;
            node.point.y = sPoint.y + unit_vec_y*j;
            path.nodes.push_back(node);
        }
        // Push the last point in there
        node.point.x = ePoint.x;
        node.point.y = ePoint.y;
        path.nodes.push_back(node);
        path.endPointIndex = path.nodes.size()-1;

        // Compute Distance
        calcTargetDistance(path, direction);

        // Compute Curvature
        // TBD

        // Add an end path. 
        addEndSegment(path, unit_vec_x, unit_vec_y, direction);
        
        // Return Path
        return path;
    }
}