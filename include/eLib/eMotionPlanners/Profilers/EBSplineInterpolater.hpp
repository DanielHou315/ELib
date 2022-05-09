#ifndef E_BSPLINE_INTERPOLATER
#define E_BSPLINE_INTERPOLATER


#include "eLib/eMotionPlanners/Profilers/EBaseInterpolater.hpp"
using namespace elib;


namespace elib{
    class EBSplineInterpolater : public EBaseInterpolater {
    public: 
        EBSplineInterpolater();
        ~EBSplineInterpolater();
        // B Spline Interpolate
        virtual PathSegment interpolatePath(vector<Point> &pointSet, bool direction);
        virtual PathSegment interpolatePath(Point &sPoint, Point &ePoint, bool direction);
    };
}

#endif