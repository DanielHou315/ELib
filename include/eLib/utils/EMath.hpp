#ifndef _E_MATH_
#define _E_MATH_


//----------------------------------------------------------

// This file contains math functions to support 
// It provides functions to transform between different forms of pose expression as well as unit conversion. 
// No API in this file is needed directly in your code. 

//----------------------------------------------------------
// Huaidian Hou @ 2022
//----------------------------------------------------------

#include <cmath>
#include <algorithm>
#include <vector>
#include <memory>
using namespace std;

#include "eLib/3rdParty/Eigen/Geometry"
#include "eLib/3rdParty/Eigen/Dense"
using namespace Eigen;

#include "eLib/utils/EUnits.hpp"
using namespace okapi;



namespace elib{
    
    #define For3 for(int i = 0; i < 3; i++)
    #define For2 for(int i = 0; i < 2; i++)

    #define NowMill pros::millis()
    #define NowMicro pros::micros()

    //--------------------------------------
    // Misc
    //--------------------------------------
    float invSqrt(float x);
    int sgn(const double &value);


    //--------------------------------------
    // Angle Bounding
    //--------------------------------------
    QAngle getBoundAngle(const QAngle& ang);
    QAngle getOppositeAngle(const QAngle& ang);
    

    //--------------------------------------
    // Euler & Quaternion
    //--------------------------------------
    // Return Euler given Eigen Quaterniond
    void quat2euler(const Eigen::Quaterniond& q, QAngle& roll, QAngle& pitch, QAngle& yaw);
    // Return certain Euler given Eigen Quaterniond
    QAngle quat2roll(const Eigen::Quaterniond& q);
    QAngle quat2pitch(const Eigen::Quaterniond& q);
    QAngle quat2yaw(const Eigen::Quaterniond& q);
    // Return Quaternion given Euler
    Quaterniond euler2quat(const QAngle& roll, const QAngle& pitch, const QAngle& yaw);
    Quaterniond averageQuaternion(Vector4f cumulative, Quaterniond newRotation, Quaterniond firstRotation, int addAmount);
    Quaterniond normalizeQuaternion(float x, float y, float z, float w);
    //Changes the sign of the quaternion components. This is not the same as the inverse.
    Quaterniond inverseSignQuaternion(Quaterniond q);
    bool areQuaternionsClose(Quaterniond q1, Quaterniond q2);

    


    //--------------------------------------
    // Rotation & Linear Distance 
    //--------------------------------------
    QLength rot2lin(const okapi::QAngle &theta, const okapi::QLength &WhlDiam, const double &GRatio);
    QAngle lin2rot(const okapi::QLength &dst, const okapi::QLength &WhlDiam, const double& GRatio);
    void turn2rot(const okapi::QAngle &theta, const okapi::QLength &WBase, 
            const okapi::QLength &WhlDiam, const double &GRat, QAngle &l_rot, QAngle &r_rot);
    
    //--------------------------------------
    // Point Distances
    //--------------------------------------
    QLength distanceFromCurrentPose(shared_ptr<OdomState> truePose, const Point& BPose);
    QLength distanceFromCurrentPose(shared_ptr<OdomState> truePose, const OdomState& BPose);
    QLength distanceApart(const Point& pt1, const Point& pt2);





} // namespace egps

#endif