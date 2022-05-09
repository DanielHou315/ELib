#ifndef E_EKF_FUSION
#define E_EKF_FUSION



//----------------------------------------------------------

// This file is based on the work of Guo-ziwei on Github.
// It provides three filtering algorithms for Position Fusion using IMU and Odometer. 
// No API in this file is needed directly in your code. 

//----------------------------------------------------------
// Huaidian Hou @ 2022
//----------------------------------------------------------
#include <memory>
using namespace std;

#include "eLib/eSLAM/localization/PoseMeasurementModel.hpp"
#include "eLib/eSLAM/localization/systemmodel.hpp"
#include "eLib/3rdParty/kalman/ExtendedKalmanFilter.hpp"

#include "eLib/utils/EMath.hpp"
#include "eLib/eSLAM/localization/EOdometry.hpp"
using namespace okapi;
using namespace elib;


namespace elib{

    typedef float T;

    // Some type shortcuts
    typedef Planar_Robot::State<T> State;
    typedef Planar_Robot::Control<T> Control;
    typedef Planar_Robot::SystemModel<T> SystemModel;
    typedef Planar_Robot::PoseMeasurement<T> PoseMeasure;
    typedef Planar_Robot::PoseMeasurementModel<T> PoseMeasureModel;



    class E_EKF{
    private:
        shared_ptr<IMUMsg> imumsg;
        shared_ptr<OdomState> truePose, odomDeltaPose;

        State x_ekf;
        Control u;
        SystemModel sys;
        PoseMeasureModel pose_measurement;

        Kalman::ExtendedKalmanFilter<State> predictor;
        Kalman::ExtendedKalmanFilter<State> ekf;

        // Predicted Position and Orientation
        State x_pred;

    public: 
        E_EKF(shared_ptr<IMUMsg> imu_src
            , shared_ptr<OdomState> trueP
            , shared_ptr<OdomState> odomDP);
        void E_EKFUpdate();
    };
}

#endif 