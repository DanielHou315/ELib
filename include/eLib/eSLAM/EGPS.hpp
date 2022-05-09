#ifndef _E_POSITIONING_TASK_
#define _E_POSITIONING_TASK_



//----------------------------------------------------------

// This file contains 2 virtual classes and 4 classes that you should initialize in your code. 
// It provides options to use both pure Odometer and IMU-Odometer fusion with two Odometer configurations available. 
// You should initialize your selected Positioning Algorithm in your Initialize() function
// Then you will only need to GetPosition() when you need to know the position. 

//----------------------------------------------------------
// Huaidian Hou @ 2022
//----------------------------------------------------------

#include <memory>
using namespace std;

#include "eLib/eSLAM/localization/EOdometry.hpp"
#include "eLib/eSLAM/localization/EEKF.hpp"
#include "eLib/utils/EMath.hpp"
#include "eLib/eDriver/EDriver.hpp"
using namespace okapi;
using namespace elib;




namespace elib{
    //-----------------------------------
    //
    // Virtual GPS class
    // Do NOT use these in your code -- only choose one of the derived classes below. 
    //
    //-----------------------------------
    class E_GPS{
    protected:
        // Mutices and Tasks 
        pros::Mutex gps_mutex;

        int lastUpdateStamp = -1;

        // Pose
        OdomState tp;
        shared_ptr<OdomState> truePose = make_shared<OdomState> (tp);
        // Delta Pose by Odometry
        OdomState odp;
        shared_ptr<OdomState> odomDeltaPose = make_shared<OdomState> (odp); 

        // Infrastructure
        QLength center2Front, center2Back, center2Left, center2Right;

        // Constructor
        E_GPS(const OdomChassisConfig& chassisconf
            , const OdomState& initposee);
        // Update True Position based on Odom Position ONLY
        void updateOdom2TruePose();
        void updateOdom2TruePose(QAngle IMUAng);
        
    public: 
        QLength dx();
        QLength dy();
        QAngle dyaw();

        QLength GPS_X();
        QLength GPS_Y();
        QAngle GPS_Yaw();


        virtual void setPose(const OdomState& initpose);
        virtual void setPose(const QLength& _x, const QLength& _y, const QAngle& _theta);

        shared_ptr<OdomState> getStatePtr();

        // Debug
        virtual void getOdomWheelDist(QLength& lDist, QLength& rDist) = 0;
        virtual void clearOdomWheelDist() = 0;
    };










    //----------------------------------------------------------
    // Two Wheel Odometer GPS
    //----------------------------------------------------------
    // Two Wheel Odometer
    // No IMU
    // No Fusion
    //----------------------------------------------------------
    class DuoOdom_GPS : public E_GPS{
    private:
        TwoEncOdometer DuoOdom;
        pros::Task E_GPS_Updater;
        // Task to Update Position with a consistent frequency
        // Start of loop time, used to maintain consistent update frequency
        void updatePosition();

    public: 
        // WheelSep -- Wheel Separation between Left and Right Wheel
        DuoOdom_GPS(const TrackingWheelConfig& lconf
                    , const TrackingWheelConfig& rconf
                    , const OdomChassisConfig& chassisconf 
                    , const OdomState& initposee);
        bool isCalibrated();
        virtual void getOdomWheelDist(QLength& lDist, QLength &rDist);
        virtual void clearOdomWheelDist();
    };










    //----------------------------------------------------------
    // Two Wheel IMU Extended Kalman Filter GPS
    //----------------------------------------------------------
    // Two Wheel Odometer
    // One IMU
    // EKF Fusion 
    //----------------------------------------------------------
    class DuoOdom_IMU_GPS : public E_GPS{
    private:
        TwoEncOdometer DuoOdom;

        EIMU imu1;
        IMUMsg imu_msg;
        shared_ptr<IMUMsg> imu_msg_ptr = make_shared<IMUMsg>(imu_msg);

        E_EKF EKFUpdater;
        // Task
        pros::Task E_GPS_Updater;

        // Task to Update Position with a consistent frequency
        // Start of loop time, used to maintain consistent update frequency
        void updatePosition();  

    public: 
        DuoOdom_IMU_GPS(const IMUConfig& imuconf
                        , const TrackingWheelConfig& lconf
                        , const TrackingWheelConfig& rconf
                        , const OdomChassisConfig& chassisconf
                        , const OdomState& initposee);
        bool isCalibrated();
        virtual void getOdomWheelDist(QLength& lDist, QLength& rDist);
        virtual void clearOdomWheelDist();

        virtual void setPose(const OdomState& initpose);
        virtual void setPose(const QLength& _x, const QLength& _y, const QAngle& _theta);
    };












    //----------------------------------------------------------
    // Two Wheel IMU Extended Kalman Filter GPS
    //----------------------------------------------------------
    // Two Wheel Odometer
    // Two IMU
    // EKF Fusion 
    //----------------------------------------------------------
    class DuoOdom_IMU2_GPS : public E_GPS{
    private:
        TwoEncOdometer DuoOdom;

        EIMU imu1;
        EIMU imu2;
        IMUMsg imu1_msg, imu2_msg, imu_msg;
        shared_ptr<IMUMsg> imu_msg_ptr = make_shared<IMUMsg>(imu1_msg);

        E_EKF EKFUpdater;
        pros::Task E_GPS_Updater;

        // Task to Update Position with a consistent frequency
        // Start of loop time, used to maintain consistent update frequency
        void updatePosition();  

    public: 
        // WheelSep -- Wheel Separation between Left and Right Wheel
        DuoOdom_IMU2_GPS(const IMUConfig& imu1conf
                        , const IMUConfig& imu2conf
                        , const TrackingWheelConfig& lconf
                        , const TrackingWheelConfig& rconf
                        , const OdomChassisConfig& chassisconf
                        , const OdomState& initposee);
        bool isCalibrated();
        virtual void getOdomWheelDist(QLength& lDist, QLength& rDist);
        virtual void clearOdomWheelDist();


        virtual void setPose(const OdomState& initpose);
        virtual void setPose(const QLength& _x, const QLength& _y, const QAngle& _theta);
    };









    //----------------------------------------------------------
    // Three Wheel Odometer GPS
    //----------------------------------------------------------
    // Three Wheel Odometer
    // No IMU
    // No Fusion
    //----------------------------------------------------------
    class TriOdom_GPS : public E_GPS{
    private:
        ThreeEncOdometer TriOdom;
        pros::Task E_GPS_Updater;
        // Task to Update Position with a consistent frequency
        // Start of loop time, used to maintain consistent update frequency
        void updatePosition();

    public: 
        // WheelSep -- Wheel Separation between Left and Right Wheel
        // BtoCDist -- Back Wheel to Center Distance (The Center is at the center of the robot between axes of rubber wheels)
        TriOdom_GPS(const TrackingWheelConfig& lconf
                    , const TrackingWheelConfig& rconf
                    , const TrackingWheelConfig& bconf
                    , const OdomChassisConfig& chassisconf
                    , const OdomState& initposee);
        bool isCalibrated();
        virtual void getOdomWheelDist(QLength& lDist, QLength& rDist, QLength& bDist);
        virtual void clearOdomWheelDist();
    };










    //----------------------------------------------------------
    // Three Wheel IMU Extended Kalman Filter GPS
    //----------------------------------------------------------
    // Three Wheel Odometer
    // One IMU
    // EKF Fusion 
    //----------------------------------------------------------
    class TriOdom_IMU_GPS : public E_GPS{
    private:
        ThreeEncOdometer TriOdom;

        EIMU imu1;
        IMUMsg imu_msg;
        shared_ptr<IMUMsg> imu_msg_ptr = make_shared<IMUMsg>(imu_msg);
        
        E_EKF EKFUpdater;
        pros::Task E_GPS_Updater;
        // Task to Update Position with a consistent frequency
        // Start of loop time, used to maintain consistent update frequency
        void updatePosition();
        
    public: 
        // WheelSep -- Wheel Separation between Left and Right Wheel
        // BtoCDist -- Back Wheel to Center Distance (The Center is at the center of the robot between axes of rubber wheels)
        TriOdom_IMU_GPS(const IMUConfig& imuconf
                        , const TrackingWheelConfig& lconf
                        , const TrackingWheelConfig& rconf
                        , const TrackingWheelConfig& bconf
                        , const OdomChassisConfig& chassisconf
                        , const OdomState& initposee);
        bool isCalibrated();
        virtual void getOdomWheelDist(QLength& lDist, QLength& rDist, QLength& bDist);
        virtual void clearOdomWheelDist();


        virtual void setPose(const OdomState& initpose);
        virtual void setPose(const QLength& _x, const QLength& _y, const QAngle& _theta);
    };

} // namespace elib


#endif