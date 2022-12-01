#include "eLib/eSLAM/localization/EOdometry.hpp"
#include "eLib/utils/EMath.hpp"
using namespace elib;
using namespace okapi;



namespace elib{
    BaseOdometer::BaseOdometer(shared_ptr<OdomState> dp){deltaPose = dp;}

    void BaseOdometer::clearWheelDist(){
        prevLinearDistance[0] = 0_m;
        prevLinearDistance[1] = 0_m;
        prevLinearDistance[2] = 0_m;
    }




    //---------------------------------------
    // Two Wheel Odometer Functions
    //---------------------------------------
    TwoEncOdometer::TwoEncOdometer(const TrackingWheelConfig& lconf
                                , const TrackingWheelConfig& rconf
                                , const OdomChassisConfig& chassisconf
                                , shared_ptr<OdomState> deltaP)
                                : leftRotationSensor(lconf.rotation_cfg)
                                , rightRotationSensor(rconf.rotation_cfg)
                                , BaseOdometer(deltaP)
    {
        wheelSensors[0] = &leftRotationSensor;
        wheelSensors[1] = &rightRotationSensor;

        // Tracking Wheel Parameters
        For3{trackingWheelRadius[i] = lconf.diameter/2;}
        leftRightSeparation = chassisconf.center2LTrackingWheel + chassisconf.center2RTrackingWheel;
        return;
    }


    // Update Odometer Value
    void TwoEncOdometer::updateDeltaPose(){
        odomMutex.take(20);
        delta_t = pros::millis() - lastUpdateTime;

        For2{
            currentReading[i] = 1_deg * wheelSensors[i]->getPosition() / 100;
            deltaLinearDistance[i] = currentReading[i].convert(radian) * trackingWheelRadius[i] - prevLinearDistance[i];
            currentLinearVelocity[i] = deltaLinearDistance[i] / (1_ms*delta_t);
        }

        // Start Odometry Calculation
        // If left == right, the robot go straight line
        if (deltaLinearDistance[0] == deltaLinearDistance[1]){
            deltaPose->x = 0_in;
            deltaPose->y = deltaLinearDistance[0];
            deltaPose->theta = 0_rad;
        }
        // Otherwise, calculate as if the robot went through a curve
        else{
            deltaPose->theta = 1_rad * (deltaLinearDistance[1] - deltaLinearDistance[0]) / leftRightSeparation;
            QLength r = 1_m * (deltaLinearDistance[0] + deltaLinearDistance[1]).convert(meter) / deltaPose->theta.convert(radian) / 2;                     //wbY subject to change, not always going more right//
            deltaPose->x = r * (cos(deltaPose->theta.convert(radian)) - 1);
            deltaPose->y = r * sin(deltaPose->theta);
        }

        // Update Current reading into the PrevReading array for next iteration
        For2{prevLinearDistance[i] += deltaLinearDistance[i];}

        lastUpdateTime += delta_t;
        odomMutex.give();
    }

    void TwoEncOdometer::getWheelDist(QLength& l, QLength& r){
        l += deltaLinearDistance[0];
        r += deltaLinearDistance[1];
    }
    void TwoEncOdometer::clearWheelDist(){
        prevLinearDistance[0] = 0_m;
        prevLinearDistance[1] = 0_m;
        prevLinearDistance[2] = 0_m;
    }




















    //---------------------------------------
    // Three Wheel Odometer Functions
    //---------------------------------------
    ThreeEncOdometer::ThreeEncOdometer(const TrackingWheelConfig& lconf
                                    , const TrackingWheelConfig& rconf
                                    , const TrackingWheelConfig& bconf
                                    , const OdomChassisConfig& chassisconf
                                    , shared_ptr<OdomState> deltaP) : 
                                    leftRotationSensor(lconf.rotation_cfg), 
                                    rightRotationSensor(rconf.rotation_cfg), 
                                    backRotationSensor(bconf.rotation_cfg), 
                                    BaseOdometer(deltaP)
    {   
        wheelSensors[0] = &leftRotationSensor;
        wheelSensors[1] = &rightRotationSensor;
        wheelSensors[1] = &backRotationSensor;

        // Tracking Wheel Parameters
        For3{trackingWheelRadius[i] = lconf.diameter/2;}
        leftRightSeparation = chassisconf.center2LTrackingWheel + chassisconf.center2RTrackingWheel;
        backCenterSeparation = chassisconf.center2BTrackingWheel;
        return;
    }

    // Update Odometer Value
    void ThreeEncOdometer::updateDeltaPose(){
        odomMutex.take(20);
        delta_t = pros::millis() - lastUpdateTime;

        // Update this round encoder reading in degrees
        For3{
            currentReading[i] = 1_deg * wheelSensors[i]->getPosition() / 100;
            deltaLinearDistance[i] = currentReading[i].convert(radian) * trackingWheelRadius[i] - prevLinearDistance[i];
            currentLinearVelocity[i] = deltaLinearDistance[i] / (1_ms*delta_t);
        }

        // Start Odometry Calculation
        // If left == right, the robot go straight line
        if (deltaLinearDistance[0] == deltaLinearDistance[1]){
            deltaPose->x = deltaLinearDistance[2];
            deltaPose->y = deltaLinearDistance[0];
            deltaPose->theta = 0_rad;
        }

        // Otherwise, calculate as if the robot went through a curve
        else{
            // Find the rotation of the robot
            deltaPose->theta = 1_rad * (deltaLinearDistance[1] - deltaLinearDistance[0]) / leftRightSeparation;

            // Subtract the natural rotation that comes with the rear wheel to get the actual wheel. 
            deltaLinearDistance[2] -= deltaPose->theta.convert(radian) * backCenterSeparation;

            // Update Position
            // Find radius of rotation
            QLength r = (deltaLinearDistance[0] + deltaLinearDistance[1]) / deltaPose->theta.convert(radian) / 2;
            deltaPose->x = r * (cos(deltaPose->theta.convert(radian)) - 1) + cos(deltaPose->theta.convert(radian) / 2) * deltaLinearDistance[2];
            deltaPose->y = r * sin(deltaPose->theta) + sin(deltaPose->theta / 2) * deltaLinearDistance[2];
        }

        // Update Current reading into the PrevReading array for next iteration
        For3{prevLinearDistance[i] += deltaLinearDistance[i];}

        lastUpdateTime += delta_t;
        odomMutex.give();
    }

    void ThreeEncOdometer::getWheelDist(QLength& l, QLength& r, QLength& b){
        l = prevLinearDistance[0];
        r = prevLinearDistance[1];
        b = prevLinearDistance[2];
    }
    void ThreeEncOdometer::clearWheelDist(){
        prevLinearDistance[0] = 0_m;
        prevLinearDistance[1] = 0_m;
        prevLinearDistance[2] = 0_m;
    }


} // namespace lib

