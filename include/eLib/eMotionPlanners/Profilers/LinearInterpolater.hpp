#ifndef E_LINEAR_INTERPOLATER
#define E_LINEAR_INTERPOLATER

#include <vector>
using namespace std;

#include "eLib/eMotionPlanners/Profilers/BaseInterpolater.hpp"
using namespace elib;


namespace elib{

    class ELinearInterpolater : public EBaseInterpolater {
    public: 
        ELinearInterpolater();
        ~ELinearInterpolater();
        // Linear Interpolate
        virtual PathSegment interpolatePath(vector<Point> &pointSet, bool direction = false);
        virtual PathSegment interpolatePath(Point &sPoint, Point &ePoint, bool direction = false);
    };
}

#endif