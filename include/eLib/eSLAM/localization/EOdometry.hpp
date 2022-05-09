#ifndef E_ODOMETRY
#define E_ODOMETRY



//----------------------------------------------------------

// This file is based on my original work in derivation of Odometry math
// It provides two Odometer models, with 2 and 3 tracking wheels respectively. 
// No API in this file is needed directly in your code. 

//----------------------------------------------------------
// Huaidian Hou @ 2022
//----------------------------------------------------------



#include "eLib/utils/EMath.hpp"
#include "eLib/eDriver/EDriver.hpp"
using namespace elib;
using namespace okapi;




namespace elib{

    //----------------------------------------------------------
    // Tracking Wheel Configuration 
    //----------------------------------------------------------
    struct TrackingWheelConfig{
        // Port of Rotation Sensor
        RotationConfig rotation_cfg;
        // diameter of tracking wheel  --  the usual definition of VEX wheels
        QLength diameter;

        TrackingWheelConfig(const int& _p, const bool& _rev, const QLength& _qdiam){
            rotation_cfg.port = _p;
            rotation_cfg.reversed = _rev;
            diameter = _qdiam;
        }
        TrackingWheelConfig(const RotationConfig rot_cfg, const QLength& _qdiam){
            rotation_cfg = rot_cfg;
            diameter = _qdiam;
        }
    };


    // Odom Chassis Config
    /// Drive Overall Sizer
    // - C2F
    // - C3B
    // - C2L
    // - C2R
    // Center to Tracking Wheel
    // - C2LT
    // - C2RT
    // - C2BT = 0
    struct OdomChassisConfig{
        QLength center2Front;
        QLength center2Back;
        QLength center2Left;
        QLength center2Right;

        QLength center2LTrackingWheel;
        QLength center2RTrackingWheel;
        QLength center2BTrackingWheel;
        
        OdomChassisConfig(const QLength& c2F, const QLength& c2B, const QLength& c2L, const QLength& c2R, 
                        const QLength& c2LT, const QLength& c2RT, const QLength& c2BT = 0_m){
            center2Front = c2F;
            center2Back = c2B;
            center2Left = c2L;
            center2Right = c2R;

            center2LTrackingWheel = c2LT;
            center2RTrackingWheel = c2RT;
            center2BTrackingWheel = c2BT;
        }
    };







    // ------------------------------------------
    // Base Odometer Class
    // - Foundation for both 2-wheel and 3-wheel Odometers
    // 
    // @params
    // 1. Global Timing [QTime]
    //   -- shared_ptr to delta_time
    //   -- shared_ptr to last_time
    // 2. Global Pose [OdomPose]
    //   -- shared_ptr to truePose, which records what the fused pose is
    //   -- shared_ptr to deltaPose, which records the Odometers recorded change in pose during delta_t
    // 3. Chassis Configuration
    //   -- center 2 front, back, left, and right [QLength]
    //      -- the center is where the IMU is, and the middle point between two rubber non-slippery wheels. 
    //   -- wheel diameters
    // 4. State Recorders & Calculators
    //   -- current reading from motors [centidegrees, see PROS documentaiton]
    //   -- previously recorded linear distance traveled [QLength]
    //   -- change in distance of each wheel in this cycle (current reading transformed to linear - previous linear distance)
    //   -- previous linear velocity [QSpeed]
    //   The following are pointers from E GPS Task
    //   -- Ptr to current linear velocity [QSpeed] (delta distance / delta_t)
    //   -- Ptr to linear acceleration [QAcceleration] ( (current vel - prev vel) / delta_t )
    // ------------------------------------------

    class BaseOdometer{
    protected:
        // Shared Pointers
        int delta_t, lastUpdateTime, procTime;
        pros::Mutex odomMutex;

        // Physical Configuration of the chassis
        ERotation * wheelSensors[3];
        QLength trackingWheelRadius[3];

        // State recorder
        // Raw Reading [centidegree]
        QAngle currentReading[3];
        // Linear Distance
        QLength prevLinearDistance[3];
        QLength deltaLinearDistance[3];
        // Linear Velocity
        QSpeed currentLinearVelocity[3];
        int calcfreq = 0;

        // Constructor
        shared_ptr<OdomState> deltaPose; 
        BaseOdometer(shared_ptr<OdomState> dp);
    public:
        virtual void clearWheelDist();
    };


    //---------------------------------------
    // Two Encoder Odometer Class
    //---------------------------------------
    class TwoEncOdometer : public BaseOdometer{
    // private:
    public: 
        //Two Rotation Sensors
        ERotation leftRotationSensor;
        ERotation rightRotationSensor;

        // Distance between Left and Right Wheel
        QLength leftRightSeparation;

    // public:
        // Constructor
        TwoEncOdometer(const TrackingWheelConfig& lconf
                    , const TrackingWheelConfig& rconf
                    , const OdomChassisConfig& chassisconf
                    , shared_ptr<OdomState> deltaP);
        // Update odometer calculations once. 
        void updateDeltaPose();
        void getWheelDist(QLength& l, QLength& r);
        virtual void clearWheelDist();
    };





    class ThreeEncOdometer : public BaseOdometer{
    private:
        //Three Rotation Sensors
        ERotation leftRotationSensor;
        ERotation rightRotationSensor;
        ERotation backRotationSensor;

        // Distance between Left and Right Wheel
        QLength  leftRightSeparation;
        // Distance between Back wheel and Center of Rotation of the robot. 
        QLength backCenterSeparation;

    public:
        // Constructor
        ThreeEncOdometer(const TrackingWheelConfig& lconf
                        , const TrackingWheelConfig& rconf
                        , const TrackingWheelConfig& bconf
                        , const OdomChassisConfig& chassisconf
                        , shared_ptr<OdomState> deltaP);
        // Update odometer calculations once. 
        void updateDeltaPose();
        void getWheelDist(QLength& l, QLength& r, QLength& b);
        virtual void clearWheelDist();
    };
}


#endif


