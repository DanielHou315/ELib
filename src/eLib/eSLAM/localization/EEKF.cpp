#include "eLib/eSLAM/localization/EEKF.hpp"
#include "eLib/eDriver/EDriver.hpp"
using namespace okapi;
using namespace elib;


#define EKF_ODOM_TEST_MODE 0



namespace elib{

E_EKF::E_EKF(shared_ptr<IMUMsg> imu_src
        , shared_ptr<OdomState> trueP
        , shared_ptr<OdomState> odomDP){
    imumsg = imu_src;
    truePose = trueP;
    odomDeltaPose = odomDP;
    

    x_ekf.setZero();
    x_ekf.qw() = 1.0;

    predictor.init(x_ekf);
    ekf.init(x_ekf);
}

void E_EKF::E_EKFUpdate(){
    // Measured Odometry Delta Pose
    PoseMeasure measure;

    truePose->x += odomDeltaPose->x;
    truePose->y += odomDeltaPose->y;
    truePose->theta += odomDeltaPose->theta;

    
    // Input Data
    // u -- IMU observed data
    u.a_x() = imumsg->linearAcceleration[0].convert(mps2);
    u.a_y() = imumsg->linearAcceleration[1].convert(mps2);
    u.a_z() = 0.0;
    u.w_x() = 0.0;
    u.w_y() = 0.0;
    u.w_z() = imumsg->angularVelocity[2].convert(radps);
    x_ekf = ekf.predict(sys, u);
    


    // -----------------------------
    // Test Portion

    #if EKF_ODOM_TEST_MODE == 0
    // Option 1
    // Measure -- Odom measured
    measure.x() = truePose->x.convert(meter);
    measure.y() = truePose->y.convert(meter);
    measure.z() = 0.0;


    #elif EKF_ODOM_TEST_MODE == 1
    // Option 2
    // Measure -- Odom measured delta pose
    measure.x() = odomDeltaPose->x.convert(meter);
    measure.y() = odomDeltaPose->y.convert(meter);
    measure.z() = 0.0;
    #endif


    // Test Portion End
    // -----------------------------



    Eigen::Quaterniond qd = euler2quat(0_rad,0_rad,truePose->theta);
    measure.qw() = qd.w();
    measure.qx() = qd.x();
    measure.qy() = qd.y();
    measure.qz() = qd.z();

    // EKF Update
    x_ekf = ekf.update(pose_measurement, measure);

    // Update to Final Product 
    truePose->x = 1_m * x_ekf.x();
    truePose->y = 1_m * x_ekf.y();

    Quaterniond q_tmp(x_ekf.qw(), x_ekf.qx(), x_ekf.qy(), x_ekf.qz());
    QAngle tmpYaw = quat2yaw(q_tmp);

    truePose->theta = getBoundAngle(tmpYaw);
}

} // namespace egps