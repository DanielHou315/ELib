#ifndef E_LINEAR_PROFILER
#define E_LINEAR_PROFILER

#include <vector>
using namespace std;

#include "eLib/eMotionPlanners/Profilers/Path.hpp"
using namespace okapi;
using namespace elib;

namespace elib{

    class EBaseInterpolater{
    protected:
        vector<Point> keypoints;
        EBaseInterpolater();
        ~EBaseInterpolater();

        // Interpolation Tools
        void calcTargetDistance(PathSegment& path, bool direction);
        void calcCurvature(PathSegment& path);
        void calcTargetVelocity(PathSegment& path, bool direction);

        void addEndSegment(PathSegment& origPath, QLength uni_vec_x, QLength uni_vec_y, bool direction = false, int len = 24);   

    public: 
        // Virtual Functions for Interpolator
        virtual PathSegment interpolatePath(vector<Point> &pointSet, bool direction) = 0;
        virtual PathSegment interpolatePath(Point &sPoint, Point &ePoint, bool direction) = 0;
    };

}

#endif