#ifndef E_IMU
#define E_IMU

#include "pros/imu.hpp"
#include "pros/rtos.hpp"

#include "eLib/utils/Math.hpp"
using namespace elib;

namespace elib{

	#define USING_MAHONY 0

	#if USING_MAHONY
    //Mahony IMU
	#define constSampleFreq	100.0f			// sample frequency in Hz
	#define twoKpDef	(2.0f * 0.5f)	    // 2 * proportional gain
	#define twoKiDef	(2.0f * 0.0f)	    // 2 * integral gain
	#endif


	struct IMUConfig{
        // port -- Port for the Positioning IMU
        int port = 0;
        // Our X, Y Z axis in correspond to the IMU's X, Y, Z coordinate. 
        // 0 -> X, 1 -> Y, 2 -> Z;
        // For Example,
        // rad_order[0] = 0 means that our first axis (X) corresponds to the (X) axis on the IMU
        // rad_order[0] = 2 means that our first axis (X) corresponds to the (Z) axis on the IMU
        int read_order[3] = {0,1,2};

        // X, Y, Z reversed or not, in our coordinate system. 
        // 1 for yes, 
        bool reversed[3] = {1,1,1};

        IMUConfig(const int& _p, const int& x_ord, const int& y_ord, const int& z_ord, const bool& x_rev, const bool& y_rev, const bool& z_rev){
            port = _p;
            read_order[0] = x_ord;
            read_order[1] = y_ord;
            read_order[2] = z_ord;
            reversed[0] = x_rev;
            reversed[1] = y_rev;
            reversed[2] = z_rev;
        }
    };





	// -------------------------------------
    // GPS
    // -------------------------------------
    struct IMUMsg{
        // Angular velocity, in radians/second
        QAngularSpeed angularVelocity[3];
        // Linar Acceleration, in meters/second2
        QAcceleration linearAcceleration[3];

        IMUMsg(){}
        ~IMUMsg(){}

        IMUMsg operator+ (const IMUMsg& right_){
            IMUMsg res;
            for (int i = 0;i < 3;i++){
                res.angularVelocity[i] = angularVelocity[i] + right_.angularVelocity[i];
                res.linearAcceleration[i] = linearAcceleration[i] + right_.linearAcceleration[i];
            }
            return res;
        }
        IMUMsg operator- (const IMUMsg& right_){
            IMUMsg res;
            for (int i = 0;i < 3;i++){
                res.angularVelocity[i] = angularVelocity[i] - right_.angularVelocity[i];
                res.linearAcceleration[i] = linearAcceleration[i] - right_.linearAcceleration[i];
            }
            return res;
        }
        IMUMsg operator* (const double& factor){
            IMUMsg res;
            for (int i = 0;i < 3;i++){
                res.angularVelocity[i] = angularVelocity[i] * factor;
                res.linearAcceleration[i] = linearAcceleration[i] * factor;
            }
            return res;
        }
        IMUMsg operator* (const int& factor){
            IMUMsg res;
            for (int i = 0;i < 3;i++){
                res.angularVelocity[i] = angularVelocity[i] * factor;
                res.linearAcceleration[i] = linearAcceleration[i] * factor;
            }
            return res;
        }
        IMUMsg operator/ (const double& factor){
            IMUMsg res;
            for (int i = 0;i < 3;i++){
                res.angularVelocity[i] = angularVelocity[i] / factor;
                res.linearAcceleration[i] = linearAcceleration[i] / factor;
            }
            return res;
        }
        IMUMsg operator/ (const int& factor){
            IMUMsg res;
            for (int i = 0;i < 3;i++){
                res.angularVelocity[i] = angularVelocity[i] / factor;
                res.linearAcceleration[i] = linearAcceleration[i] / factor;
            }
            return res;
        }
    };



	


	//--------------------------------
	// E IMU
	//--------------------------------
	
	class EIMU{
	private:
		// Common
		pros::Task IMUPullTask;
		
        // Read Order
		int readOrder[3];
		bool axisReversed[3];

        // Floor Calibration
        double floorVector[3];
        bool floorCalibrated = false;

		// Input
		// Tmp Storage
		pros::c::imu_gyro_s_t gyro_reading;
		pros::c::imu_accel_s_t acc_reading;
		double gyro_dr[3], acc_dr[3];

		// Output Variables
		// This part is not transformed into IMUMsg because Mahony filter needs it
		double angularVelocity[3];
		double linearAcceleration[3];
		
		#if USING_MAHONY
		const int sampleFreq = constSampleFreq;
		// AHRS Mahony
		QAngle roll, pitch;
		float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;					// quaternion of sensor frame relative to auxiliary frame
		float twoKp = twoKpDef;											// 2 * proportional gain (Kp)
		float twoKi = twoKiDef;											// 2 * integral gain (Ki)
		float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki  
		#endif

		// Pull IMU reading into gyro_reading and acc_reading. 
		void pullReading();

		#if USING_MAHONY
		// Mahony AHRS Updater 6DOF
		void MahonyAHRSTask();
		#endif
		

	public: 
        pros::IMU IMU;


		// Constructor
		EIMU(const IMUConfig& imuconf);

        void calibrateFloor();

		void getReading(IMUMsg& msg);

		QAcceleration a_x();
		QAcceleration a_y();
		QAcceleration a_z();

		QAngularSpeed g_x();
		QAngularSpeed g_y();
		QAngularSpeed g_z();

		QAngle getRoll();
		QAngle getPitch();
        QAngle getYaw();

		bool isBaseCalibrated();
        bool isFloorCalibrated();
	};
} // namespace elib



#endif