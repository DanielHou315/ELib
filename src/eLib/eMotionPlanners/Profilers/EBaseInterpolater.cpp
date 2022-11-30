#include "eLib/eMotionPlanners/Profilers/EBaseInterpolater.hpp"
#include "eLib/utils/EMath.hpp"
using namespace elib;
using namespace okapi;
using namespace Eigen;

namespace elib{



    // ----------------------------------------------------------
    // 
    // Base Interpolator
    // 
    // ----------------------------------------------------------
    EBaseInterpolater::EBaseInterpolater(){}
    EBaseInterpolater::~EBaseInterpolater(){}


    void EBaseInterpolater::calcTargetDistance(PathSegment& path, bool direction){
        int pathSize = path.nodes.size();
        path.nodes[pathSize-1].targetDistance = 0_m;
        // Compute Distance
        for(auto i = pathSize-2; i >= 0;i--){
            QLength dst = distanceApart(path.nodes[i].point, path.nodes[i+1].point);
            if (direction) dst *= -1;
            path.nodes[i].targetDistance = dst + path.nodes[i+1].targetDistance;
        }
    }
    void EBaseInterpolater::calcCurvature(PathSegment& path){
        for(auto i = 1; i <= path.nodes.size()-2 ;i--){
            double curvature;
            double x1 = path.nodes[i].point.x.convert(inch) + 0.00001;
            double y1 = path.nodes[i].point.y.convert(inch);
            double x2 = path.nodes[i-1].point.x.convert(inch);
            double y2 = path.nodes[i-1].point.y.convert(inch);
            double x3 = path.nodes[i+1].point.x.convert(inch);
            double y3 = path.nodes[i+1].point.y.convert(inch);

            double k1 = 0.5 * (pow(x1,2)+ pow(y1,2) - pow(x2,2)- pow(y2,2)) / (x1 - x2);
            
            double k2 = (y1 - y2)/(x1 - x2);

            double b = 0.5*(pow(x2,2) - 2 * x2 * k1 + pow(y2,2) - pow(x3,2) + 2 * x3 * k1 - pow(y3,2)) / (x3 * k2 - y3 + y2 - x2 * k2);
            double a = k1 - k2 * b;

            double r = sqrt(pow(x1-a,2) + pow(y1-b,2));
            curvature = 1/r;
            if (isnan(curvature)) curvature = 0.0;
            path.nodes[i].curvature = curvature;
        }
        path.nodes[0].curvature = 0.0;
        path.nodes[path.size()-1].curvature = 0.0;
    }
    void EBaseInterpolater::calcTargetVelocity(PathSegment& path, bool direction){}


    // Add an end path. 
    void EBaseInterpolater::addEndSegment(PathSegment& origPath, QLength uni_vec_x, QLength uni_vec_y, bool direction, int len){
        PathNode node;
        PathNode endPnt = origPath.nodes[origPath.size()-1];
        // Find Unit Length
        QLength dl = 1_m * hypot(uni_vec_x.convert(meter), uni_vec_y.convert(meter));
        // Make Points
        for (int i = 1;i < len;i++){
            node.point.x = endPnt.point.x + uni_vec_x*i;
            node.point.y = endPnt.point.y + uni_vec_y*i;
            // Add target Distance
            node.targetDistance = (-1) * i * dl;
            if (direction) node.targetDistance *= -1;
            origPath.nodes.push_back(node);
        }
        return;
    }







    

} // namespace elib