#include "eLib/eSLAM/EGPS.hpp"
#include "pros/misc.hpp"
using namespace elib;
using namespace okapi;

namespace elib{

    // E_GPS Constructor
    // @params
    // -- OdomState initial robot pose on field
    //    -- this should change from setup to setup
    E_GPS::E_GPS(const OdomChassisConfig& chassisconf
                , const OdomState& initposee) 
                : tp(initposee)
                , odp(0_m,0_m, 0_rad)
    {   
        center2Front = chassisconf.center2Front;
        center2Back = chassisconf.center2Back;
        center2Left = chassisconf.center2Left;
        center2Right = chassisconf.center2Right;
    }
    
    // Update True Pose based on Odometer Data Only
    void E_GPS::updateOdom2TruePose(){
        truePose->x += (sin(truePose->theta) * odomDeltaPose->x + cos(truePose->theta) * odomDeltaPose->y);
        truePose->y += (cos(truePose->theta) * odomDeltaPose->x + sin(truePose->theta) * odomDeltaPose->y);
        
        // Update Yaw
        QAngle tmpYaw = truePose->theta + odomDeltaPose->theta;
        truePose->theta = getBoundAngle(tmpYaw);
    }

    void E_GPS::updateOdom2TruePose(QAngle IMUAng){
        // If IMU Angle is 
        if(isnan(IMUAng.convert(degree))) return;

        truePose->x += (sin(truePose->theta) * odomDeltaPose->x + cos(truePose->theta) * odomDeltaPose->y);
        truePose->y += (cos(truePose->theta) * odomDeltaPose->x + sin(truePose->theta) * odomDeltaPose->y);
        // Update Yaw
        truePose->theta = getBoundAngle(IMUAng);
    }


    
    

    //Show Odometry Results
    QLength E_GPS::dx(){
        gps_mutex.take(10);
        return odomDeltaPose->x;
        gps_mutex.give();
    }
    QLength E_GPS::dy(){
        gps_mutex.take(10);
        return odomDeltaPose->y;
        gps_mutex.give();
    }
    QAngle E_GPS::dyaw(){
        gps_mutex.take(10);
        return odomDeltaPose->theta;
        gps_mutex.give();
    }

    QLength E_GPS::GPS_X(){
        gps_mutex.take(10);
        return truePose->x;
        gps_mutex.give();
    }
    QLength E_GPS::GPS_Y(){
        gps_mutex.take(10);
        return truePose->y;
        gps_mutex.give();
    }
    QAngle E_GPS::GPS_Yaw(){
        gps_mutex.take(10);
        return truePose->theta;
        gps_mutex.give();
    }

    void E_GPS::setPose(const OdomState& initpose){
        gps_mutex.take(10);
        truePose->x = initpose.x;
        truePose->y = initpose.y;
        truePose->theta = initpose.theta;
        gps_mutex.give();
    }
    void E_GPS::setPose(const QLength& _x, const QLength& _y, const QAngle& _theta){
        gps_mutex.take(10);
        truePose->x = _x;
        truePose->y = _y;
        truePose->theta = _theta;
        gps_mutex.give();
    }

    shared_ptr<OdomState> E_GPS::getStatePtr(){return truePose;}








    //----------------------------------------------------------
    // Two Wheel Odometer GPS
    //----------------------------------------------------------
    DuoOdom_GPS::DuoOdom_GPS(const TrackingWheelConfig& lconf
                            , const TrackingWheelConfig& rconf
                            , const OdomChassisConfig& chassisconf
                            , const OdomState& initposee)
                            : DuoOdom(lconf, rconf, chassisconf, odomDeltaPose)
                            , E_GPS(chassisconf, initposee)
                            , E_GPS_Updater(std::bind(&DuoOdom_GPS::updatePosition,this))
                            {}

    void DuoOdom_GPS::updatePosition(){
        okapi::Rate rateController;
        while(true){
            // Start New Cycle
            gps_mutex.take(20);
            // Update Delta Pose
            DuoOdom.updateDeltaPose();
            // Update Global Pose
            updateOdom2TruePose();

            // Wrap Up
            lastUpdateStamp = NowMill;
            gps_mutex.give();
            rateController.delay(100_Hz);
        }
    }
    void DuoOdom_GPS::getOdomWheelDist(QLength& lDist, QLength& rDist){DuoOdom.getWheelDist(lDist, rDist);}
    void DuoOdom_GPS::clearOdomWheelDist(){DuoOdom.clearWheelDist();}










    //----------------------------------------------------------
    // Two Wheel IMU Extended Kalman Filter GPS
    //----------------------------------------------------------
    // Two Wheel Odometer
    // One IMU
    // EKF Fusion 
    //----------------------------------------------------------
    DuoOdom_IMU_GPS::DuoOdom_IMU_GPS(const IMUConfig& imuconf
                                    , const TrackingWheelConfig& lconf
                                    , const TrackingWheelConfig& rconf
                                    , const OdomChassisConfig& chassisconf
                                    , const OdomState& initposee)
                                    : imu1(imuconf)
                                    , DuoOdom(lconf, rconf, chassisconf, odomDeltaPose)
                                    , EKFUpdater(imu_msg_ptr, truePose, odomDeltaPose)
                                    , E_GPS(chassisconf, initposee)
                                    , E_GPS_Updater(std::bind(&DuoOdom_IMU_GPS::updatePosition,this))
                                    {imu1.IMU.set_yaw(initposee.theta.convert(degree));}
    
    void DuoOdom_IMU_GPS::updatePosition(){
        pros::delay(200);
        while(!imu1.isBaseCalibrated()) pros::delay(1);
        imu1.calibrateFloor();
        Rate rateController;
        while(true){
            gps_mutex.take(20);

            // Get IMU Messages
            imu1.getReading(imu_msg);
            // Update Delta Pose
            DuoOdom.updateDeltaPose();
            
            // Update Global Position with Fusion 
            // EKF internally handles global position increment
            // So there is no need for any updateTruePose() function. 
            EKFUpdater.E_EKFUpdate();
            
            // Wrap Up
            lastUpdateStamp = NowMill;
            gps_mutex.give();
            rateController.delay(100_Hz);
        }
    }
    bool DuoOdom_IMU_GPS::isCalibrated(){return imu1.isFloorCalibrated() && imu1.isBaseCalibrated();}
    void DuoOdom_IMU_GPS::getOdomWheelDist(QLength& lDist, QLength& rDist){DuoOdom.getWheelDist(lDist, rDist);}
    void DuoOdom_IMU_GPS::clearOdomWheelDist(){DuoOdom.clearWheelDist();}

    void DuoOdom_IMU_GPS::setPose(const OdomState& initpose){
        gps_mutex.take(10);
        truePose->x = initpose.x;
        truePose->y = initpose.y;
        truePose->theta = initpose.theta;
        imu1.IMU.set_yaw(initpose.theta.convert(degree));
        gps_mutex.give();
    }
    void DuoOdom_IMU_GPS::setPose(const QLength& _x, const QLength& _y, const QAngle& _theta){
        gps_mutex.take(10);
        truePose->x = _x;
        truePose->y = _y;
        truePose->theta = _theta;
        imu1.IMU.set_yaw(_theta.convert(degree));
        gps_mutex.give();
    }










    //----------------------------------------------------------
    // Two Wheel Two IMU Extended Kalman Filter GPS
    //----------------------------------------------------------
    // Two Wheel Odometer
    // Two IMU
    // EKF Fusion 
    //----------------------------------------------------------
    DuoOdom_IMU2_GPS::DuoOdom_IMU2_GPS(const IMUConfig& imu1conf
                                        , const IMUConfig& imu2conf
                                        , const TrackingWheelConfig& lconf
                                        , const TrackingWheelConfig& rconf
                                        , const OdomChassisConfig& chassisconf
                                        , const OdomState& initposee) 
                                        : imu1(imu1conf)
                                        , imu2(imu2conf)
                                        , DuoOdom(lconf, rconf, chassisconf, odomDeltaPose)
                                        , EKFUpdater(imu_msg_ptr, truePose, odomDeltaPose)
                                        , E_GPS(chassisconf, initposee) 
                                        , E_GPS_Updater(std::bind(&DuoOdom_IMU2_GPS::updatePosition,this))
                                        {
                                            imu1.IMU.set_yaw(initposee.theta.convert(degree) * -1);
                                            imu2.IMU.set_yaw(initposee.theta.convert(degree) * -1);
                                        }
    void DuoOdom_IMU2_GPS::updatePosition(){
        pros::delay(200);
        while(!imu1.isBaseCalibrated()) pros::delay(1);
        while(!imu2.isBaseCalibrated()) pros::delay(1);
        Rate rateController;
        // Receive compute order from watchIMUs
        while(true){
            gps_mutex.take(20);

            // get IMU Data
            QAngle a1 = getBoundAngle(-1 * imu1.getYaw());
            QAngle a2 = getBoundAngle(-1 * imu2.getYaw());

            // Update Delta Pose
            DuoOdom.updateDeltaPose();
            
            // Update Global Position with Fusion 
            // EKFUpdater.E_EKFUpdate();
            updateOdom2TruePose((a1+a2)/2);
            
            // Wrap Up
            lastUpdateStamp = NowMill;
            gps_mutex.give();
            rateController.delay(100_Hz);
        }
    }
    bool DuoOdom_IMU2_GPS::isCalibrated(){return imu1.isFloorCalibrated() && imu1.isBaseCalibrated() && imu2.isFloorCalibrated() && imu2.isBaseCalibrated();}
    void DuoOdom_IMU2_GPS::getOdomWheelDist(QLength& lDist, QLength& rDist){DuoOdom.getWheelDist(lDist, rDist);}
    void DuoOdom_IMU2_GPS::clearOdomWheelDist(){DuoOdom.clearWheelDist();}
    void DuoOdom_IMU2_GPS::setPose(const OdomState& initpose){
        gps_mutex.take(10);
        truePose->x = initpose.x;
        truePose->y = initpose.y;
        truePose->theta = initpose.theta;
        imu1.IMU.set_yaw(initpose.theta.convert(degree));
        imu2.IMU.set_yaw(initpose.theta.convert(degree));
        gps_mutex.give();
    }
    void DuoOdom_IMU2_GPS::setPose(const QLength& _x, const QLength& _y, const QAngle& _theta){
        gps_mutex.take(10);
        truePose->x = _x;
        truePose->y = _y;
        truePose->theta = _theta;
        imu1.IMU.set_yaw(_theta.convert(degree) * -1);
        imu2.IMU.set_yaw(_theta.convert(degree) * -1);
        gps_mutex.give();
    }







    //----------------------------------------------------------
    // Three Wheel Odometer GPS
    //----------------------------------------------------------
    TriOdom_GPS::TriOdom_GPS(const TrackingWheelConfig& lconf
                            , const TrackingWheelConfig& rconf
                            , const TrackingWheelConfig& bconf
                            , const OdomChassisConfig& chassisconf
                            , const OdomState& initposee) 
                            : TriOdom(lconf,rconf,bconf,chassisconf, odomDeltaPose)
                            , E_GPS(chassisconf, initposee)
                            , E_GPS_Updater(std::bind(&TriOdom_GPS::updatePosition,this))
                            {}

    // Pure Odometry Updates at 100 Hz 
    void TriOdom_GPS::updatePosition(){
        Rate rateController;
        while(true){
            // Start new cycle
            gps_mutex.take(20);
            // Update Delta Pose
            TriOdom.updateDeltaPose();
            // Update Global Pose
            updateOdom2TruePose();

            // Wrap up
            lastUpdateStamp = NowMill;
            gps_mutex.give();
            rateController.delay(100_Hz);
        }
    }
    bool TriOdom_GPS::isCalibrated(){return true;}
    void TriOdom_GPS::getOdomWheelDist(QLength& lDist, QLength& rDist, QLength& bDist){TriOdom.getWheelDist(lDist, rDist, bDist);}
    void TriOdom_GPS::clearOdomWheelDist(){TriOdom.clearWheelDist();}




    //----------------------------------------------------------
    // Three Wheel IMU Extended Kalman Filter GPS
    //----------------------------------------------------------
    // Three Wheel Odometer
    // One IMU
    // EKF Fusion 
    //----------------------------------------------------------
    TriOdom_IMU_GPS::TriOdom_IMU_GPS(const IMUConfig& imuconf
                                    , const TrackingWheelConfig& lconf
                                    , const TrackingWheelConfig& rconf
                                    , const TrackingWheelConfig& bconf
                                    , const OdomChassisConfig& chassisconf
                                    , const OdomState& initposee)
                                    : TriOdom(lconf,rconf,bconf,chassisconf, odomDeltaPose)
                                    , imu1(imuconf)
                                    , E_GPS(chassisconf, initposee)
                                    , EKFUpdater(imu_msg_ptr, truePose,odomDeltaPose)
                                    , E_GPS_Updater(std::bind(&TriOdom_IMU_GPS::updatePosition,this))
                                    {imu1.IMU.set_yaw(initposee.theta.convert(degree));}

    void TriOdom_IMU_GPS::updatePosition(){
        pros::delay(200);
        while(!imu1.isBaseCalibrated()) pros::delay(1);
        imu1.calibrateFloor();
        Rate rateController;
        while(true){
            gps_mutex.take(20);
            // Get IMU Readi g
            imu1.getReading(imu_msg);
            // Update Odometry
            TriOdom.updateDeltaPose();
            // Update Global Position with Fusion 
            EKFUpdater.E_EKFUpdate();
            // Wrap Up
            lastUpdateStamp = NowMill;
            gps_mutex.give();
            rateController.delay(100_Hz);
        }
    }
    bool TriOdom_IMU_GPS::isCalibrated(){return imu1.isFloorCalibrated() && imu1.isBaseCalibrated();}
    void TriOdom_IMU_GPS::getOdomWheelDist(QLength& lDist, QLength& rDist, QLength& bDist){TriOdom.getWheelDist(lDist, rDist, bDist);}
    void TriOdom_IMU_GPS::clearOdomWheelDist(){TriOdom.clearWheelDist();}

    void TriOdom_IMU_GPS::setPose(const OdomState& initpose){
        gps_mutex.take(10);
        truePose->x = initpose.x;
        truePose->y = initpose.y;
        truePose->theta = initpose.theta;
        imu1.IMU.set_yaw(initpose.theta.convert(degree));
        gps_mutex.give();
    }
    void TriOdom_IMU_GPS::setPose(const QLength& _x, const QLength& _y, const QAngle& _theta){
        gps_mutex.take(10);
        truePose->x = _x;
        truePose->y = _y;
        truePose->theta = _theta;
        imu1.IMU.set_yaw(_theta.convert(degree));
        gps_mutex.give();
    }


} // namespace elib

