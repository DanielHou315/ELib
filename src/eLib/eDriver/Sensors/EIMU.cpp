#include "eLib/eDriver/Sensors/EIMU.hpp"
using namespace elib;

using namespace Eigen;

namespace elib{

    EIMU::EIMU(const IMUConfig& imuconf)
			: IMU(imuconf.port)
			, IMUPullTask(std::bind(&EIMU::pullReading, this)) 
			#if USING_MAHONY
			, IMUMahonyTask(std::bind(&EIMU::MahonyAHRSTask, this))
			#endif
	{
		#if USING_MAHONY
		IMUMahonyTask.suspend();
		#endif

		floorVector[2] = 1.0;
		For3{
			readOrder[i] = imuconf.read_order[i];
			axisReversed[i] = imuconf.reversed[i];
		}
		#if USING_MAHONY
		roll = 0_deg;
		pitch = 0_deg;
		#endif

		#if USING_MAHONY
		IMUMahonyTask.resume();
		#endif
	
	}

	void EIMU::pullReading(){
		while(IMU.is_calibrating()) pros::delay(1);
		okapi::Rate rateController;
		while(true){
			// As long as imu itself is not calibrating
			if (IMU.is_calibrating()){
				pros::delay(1);
				continue;
			}
			// Get Raw Data
			acc_reading = IMU.get_accel();
			gyro_reading = IMU.get_gyro_rate();
			gyro_dr[0] = gyro_reading.x - floorVector[0];												// Gyro in deg/s
			gyro_dr[1] = gyro_reading.y - floorVector[1];
			gyro_dr[2] = gyro_reading.z - floorVector[2];
			acc_dr[0] = acc_reading.x;													// Acceleration in G 
			acc_dr[1] = acc_reading.y;
			acc_dr[2] = acc_reading.z;

			For3{
				angularVelocity[i] = gyro_dr[readOrder[i]] * axisReversed[i];			// degps
				linearAcceleration[i] = acc_dr[readOrder[i]] * axisReversed[i] * 9.8;	// mps2
			}
			rateController.delay(100_Hz);
		}
	}

	void EIMU::calibrateFloor(){
		// Calibrate Floor
		// Calibrate the orientation of gravity to adjust future IMU readings
		double ta[3];
		for(int t = 0;t < 100;t++){
			For3{ta[i] += linearAcceleration[i]/100;}
			pros::delay(10);
		}
		For3{floorVector[i] = ta[i];}
		floorCalibrated = true;
	}

	#if USING_MAHONY
    //============================================================================================
	// MahonyAHRS
	//=====================================================================================================
	//
	// Madgwick's implementation of Mayhony's AHRS algorithm.
	// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
	//
	//=====================================================================================================
	void EIMU::MahonyAHRSTask() {
		while(pros::Task::notify_take(true, TIMEOUT_MAX) && !isCalibrating()){
			startLoopTime = pros::millis();

			float recipNorm;
			float halfvx, halfvy, halfvz;
			float halfex, halfey, halfez;
			float qa, qb, qc;

			// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
			if(!((linearAcceleration[0] == 0.0f) && (linearAcceleration[1] == 0.0f) && (linearAcceleration[2] == 0.0f))) {

				// Normalise accelerometer measurement
				recipNorm = invSqrt(linearAcceleration[0] * linearAcceleration[0] + linearAcceleration[1] * linearAcceleration[1] + linearAcceleration[2] * linearAcceleration[2]);
				linearAcceleration[0] *= recipNorm;
				linearAcceleration[1] *= recipNorm;
				linearAcceleration[2] *= recipNorm;        

				// Estimated direction of gravity and vector perpendicular to magnetic flux
				halfvx = q1 * q3 - q0 * q2;
				halfvy = q0 * q1 + q2 * q3;
				halfvz = q0 * q0 - 0.5f + q3 * q3;
			
				// Error is sum of cross product between estimated and measured direction of gravity
				halfex = (linearAcceleration[1] * halfvz - linearAcceleration[2] * halfvy);
				halfey = (linearAcceleration[2] * halfvx - linearAcceleration[0] * halfvz);
				halfez = (linearAcceleration[0] * halfvy - linearAcceleration[1] * halfvx);

				// Compute and apply integral feedback if enabled
				if(twoKi > 0.0f) {
					integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
					integralFBy += twoKi * halfey * (1.0f / sampleFreq);
					integralFBz += twoKi * halfez * (1.0f / sampleFreq);
					angularVelocity[0] += integralFBx;	// apply integral feedback
					angularVelocity[1] += integralFBy;
					angularVelocity[2] += integralFBz;
				}
				else {
					integralFBx = 0.0f;	// prevent integral windup
					integralFBy = 0.0f;
					integralFBz = 0.0f;
				}

				// Apply proportional feedback
				angularVelocity[0] += twoKp * halfex;
				angularVelocity[1] += twoKp * halfey;
				angularVelocity[2] += twoKp * halfez;
			}
			
			// Integrate rate of change of quaternion
			angularVelocity[0] *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
			angularVelocity[1] *= (0.5f * (1.0f / sampleFreq));
			angularVelocity[2] *= (0.5f * (1.0f / sampleFreq));
			qa = q0;
			qb = q1;
			qc = q2;
			q0 += (-qb * angularVelocity[0] - qc * angularVelocity[1] - q3 * angularVelocity[2]);
			q1 += (qa * angularVelocity[0] + qc * angularVelocity[2] - q3 * angularVelocity[1]);
			q2 += (qa * angularVelocity[1] - qb * angularVelocity[2] + q3 * angularVelocity[0]);
			q3 += (qa * angularVelocity[2] + qb * angularVelocity[1] - qc * angularVelocity[0]);
			// Normalise quaternion
			recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
			q0 *= recipNorm;
			q1 *= recipNorm;
			q2 *= recipNorm;
			q3 *= recipNorm;

			Eigen::Quaterniond qtemp(q0,q1,q2,q3);
			QAngle dummyyaw;
			quat2euler(qtemp, roll, pitch, dummyyaw);
		}
	}
	#endif


	void EIMU::getReading(IMUMsg& msg){
		For3{
			msg.linearAcceleration[i] = 1_mps2 * linearAcceleration[i];
			msg.angularVelocity[i] = 1_rpm * angularVelocity[i] / 6;
		}
	}

	QAcceleration EIMU::a_x(){return 1_mps2 * linearAcceleration[0];}
	QAcceleration EIMU::a_y(){return 1_mps2 * linearAcceleration[1];}
	QAcceleration EIMU::a_z(){return 1_mps2 * linearAcceleration[2];}

	QAngularSpeed EIMU::g_x(){return 1_rpm * angularVelocity[0] / 6;}
	QAngularSpeed EIMU::g_y(){return 1_rpm * angularVelocity[1] / 6;}
	QAngularSpeed EIMU::g_z(){return 1_rpm * angularVelocity[2] / 6;}

	QAngle EIMU::getRoll(){return 1_deg * IMU.get_roll();}
	QAngle EIMU::getPitch(){return 1_deg * IMU.get_pitch();}
	QAngle EIMU::getYaw(){return 1_deg * IMU.get_yaw();}

	bool EIMU::isBaseCalibrated(){return !IMU.is_calibrating();}
    bool EIMU::isFloorCalibrated(){return floorCalibrated;}

}