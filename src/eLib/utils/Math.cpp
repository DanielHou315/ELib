#include "eLib/utils/Math.hpp"
using namespace okapi;
using namespace elib;

namespace elib{

    float invSqrt(float x) {
        float halfx = 0.5f * x;
        float y = x;
        long i = *(long*)&y;
        i = 0x5f3759df - (i>>1);
        y = *(float*)&i;
        y = y * (1.5f - (halfx * y * y));
        return y;
    }


    QAngle getBoundAngle(const QAngle& ang){
        if (ang <= -1_rad* M_PI) return ang + 360_deg;
        else if (ang > 1_rad * M_PI) return ang - 360_deg;
        return ang;
    }

    QAngle getOppositeAngle(const QAngle& ang){
        QAngle rtn = ang-180_deg;
        return getBoundAngle(rtn);
    }


    int sgn(const double &value){return value > 0 ? 1 : (value < 0 ? -1 : 0);}

    QLength rot2lin(const okapi::QAngle &theta, const okapi::QLength &WhlDiam, const double &GRatio){
        return 1_m * WhlDiam.convert(okapi::meter) * theta.convert(okapi::radian) * GRatio / 2;
    }

    QAngle lin2rot(const okapi::QLength &dst, const okapi::QLength &WhlDiam, const double& GRatio){
        return 1_rad * dst.convert(okapi::meter) / WhlDiam.convert(okapi::meter) * 2 / GRatio;
    }

    void turn2rot(const okapi::QAngle &theta, const okapi::QLength &WBase, 
                const okapi::QLength &WhlDiam, const double &GRat, QAngle &l_rot, QAngle &r_rot)
    {
        okapi::QLength LinDst = WBase * theta.convert(okapi::radian) / 2;
        l_rot = (-1) * lin2rot(LinDst, WhlDiam, GRat);
        r_rot = lin2rot(LinDst, WhlDiam, GRat);
        return;
    }

    // Algorithm adopted from https://zhuanlan.zhihu.com/p/55790406
    void quat2euler(const Eigen::Quaterniond& q, QAngle& roll, QAngle& pitch, QAngle& yaw){
        // roll (x-axis rotation)
        double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
        double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
        roll = 1_rad * atan2(sinr_cosp, cosr_cosp);

        // pitch (y-axis rotation)
        double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
        if (fabs(sinp) >= 1)
        pitch = 1_rad * copysign(M_PI / 2, sinp); // use 90 degrees if out of range
        else
        pitch = 1_rad * asin(sinp);

        // yaw (z-axis rotation)
        double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
        double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()); 
        yaw = 1_rad * atan2(siny_cosp, cosy_cosp);
    }

    // Algorithm adopted from https://zhuanlan.zhihu.com/p/55790406
    QAngle quat2yaw(const Eigen::Quaterniond& q){
        // yaw (z-axis rotation)
        double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
        double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()); 
        return 1_rad * atan2(siny_cosp, cosy_cosp);
    }

    // Algorithm adopted from https://zhuanlan.zhihu.com/p/55790406
    QAngle quat2roll(const Eigen::Quaterniond& q){
        // roll (x-axis rotation)
        double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
        double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
        return 1_rad * atan2(sinr_cosp, cosr_cosp);
    }

    // Algorithm adopted from https://zhuanlan.zhihu.com/p/55790406
    QAngle quat2pitch(const Eigen::Quaterniond& q){
        // pitch (y-axis rotation)
        double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
        if (fabs(sinp) >= 1)
        return 1_rad * copysign(M_PI / 2, sinp); // use 90 degrees if out of range
        else
        return 1_rad * asin(sinp);
    }


    Quaterniond euler2quat(const QAngle& roll, const QAngle& pitch, const QAngle& yaw){
        Eigen::Vector3d ea0(yaw.convert(radian),pitch.convert(radian),roll.convert(radian)); 
        Eigen::Matrix3d R; 
        R = Eigen::AngleAxisd(ea0(0), ::Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(ea0(1), ::Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(ea0(2), ::Eigen::Vector3d::UnitX()); 
        
        Eigen::Quaterniond q(R);
        return q;
    }


    // ------------------------------
    // The following averaging quaternion methods is adapted from 
    // https://stackoverflow.com/questions/12374087/average-of-multiple-quaternions
    // ------------------------------
    
    //Get an average (mean) from more then two quaternions (with two, slerp would be used).
    //Note: this only works if all the quaternions are relatively close together.
    //Usage: 
    //-Cumulative is an external Vector4 which holds all the added x y z and w components.
    //-newRotation is the next rotation to be added to the average pool
    //-firstRotation is the first quaternion of the array to be averaged
    //-addAmount holds the total amount of quaternions which are currently added
    //This function returns the current average quaternion
    Quaterniond averageQuaternion(Vector4f cumulative, Quaterniond newRotation, Quaterniond firstRotation, int addAmount){

        float w = 0.0f;
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;

        //Before we add the new rotation to the average (mean), we have to check whether the quaternion has to be inverted. Because
        //q and -q are the same rotation, but cannot be averaged, we have to make sure they are all the same.
        if(!areQuaternionsClose(newRotation, firstRotation)){newRotation = inverseSignQuaternion(newRotation);}
        //Average the values
        float addDet = 1.0/(float)addAmount;
        cumulative.w() += newRotation.w();
        w = cumulative.w() * addDet;
        cumulative.x() += newRotation.x();
        x = cumulative.x() * addDet;
        cumulative.y() += newRotation.y();
        y = cumulative.y() * addDet;
        cumulative.z() += newRotation.z();
        z = cumulative.z() * addDet;      

        //note: if speed is an issue, you can skip the normalization step
        return normalizeQuaternion(x, y, z, w);
    }

    Quaterniond normalizeQuaternion(float x, float y, float z, float w){

        float lengthD = 1.0f / (w*w + x*x + y*y + z*z);
        w *= lengthD;
        x *= lengthD;
        y *= lengthD;
        z *= lengthD;

        return Quaterniond(x, y, z, w);
    }

    //Changes the sign of the quaternion components. This is not the same as the inverse.
    Quaterniond inverseSignQuaternion(Quaterniond q){
        Quaterniond p(-q.x(), -q.y(), -q.z(), -q.w());
        return p;
    }

    //Returns true if the two input quaternions are close to each other. This can
    //be used to check whether or not one of two quaternions which are supposed to
    //be very similar but has its component signs reversed (q has the same rotation as
    //-q)
    bool areQuaternionsClose(Quaterniond q1, Quaterniond q2){return (q1.dot(q2) < 0.0f);}




    // Get Direct Distance from current pose to target pose
    QLength distanceFromCurrentPose(shared_ptr<OdomState> truePose, const Point& BPose){return 1_m * hypot(truePose->x.convert(meter) - BPose.x.convert(meter), truePose->y.convert(meter) - BPose.y.convert(meter));}
    QLength distanceFromCurrentPose(shared_ptr<OdomState> truePose, const OdomState& BPose){return 1_m * hypot(truePose->x.convert(meter) - BPose.x.convert(meter), truePose->y.convert(meter) - BPose.y.convert(meter));}
    QLength distanceApart(const Point& pt1, const Point& pt2){return 1_m * hypot(pt1.x.convert(meter) - pt2.x.convert(meter), pt1.y.convert(meter) - pt2.y.convert(meter));}

} // namespace elib