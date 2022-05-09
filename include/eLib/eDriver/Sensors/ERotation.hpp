#ifndef E_ROTATION
#define E_ROTATION

#include "pros/rotation.hpp"
#include "pros/rtos.hpp"

namespace elib{

    struct RotationConfig{
        int port;
        bool reversed;
        int data_rate;

        RotationConfig(){
            reversed = 0;
            data_rate = 10;
        }
        RotationConfig(const int& p, const bool& rev, const int& dr = 10){
            port = p;
            reversed = rev;
            data_rate = dr;
        }
    };


    
    //--------------------------------
	// E Rotaiton Sensor
	//--------------------------------
	class ERotation
	{
		#define ROTATION_CALCULATE 1
	private:
		pros::Rotation rotationSensor;
		#if ROTATION_CALCULATE
		pros::Task RotationTask;

		int prevPosition;
		int currentPosition;

		int currentTimeStamp;
		int prevTimeStamp;
		double deltaTime;

		double prevVelocity;
		double currentVelocity;
		double Acceleration;

		// Get the reading and perform velocity / acceleration calculations
		void pullValue();
		#endif
		
	public:
		// Rotation Sensor Object Constructor
		ERotation(const RotationConfig& rot_cfg);

		// Set the position of a rotaiton sensor to a value
		void setPosition(int p);
		void setDataRate(int r);

		// Get current reading of rotation sensor
		int getPosition();
		double getAngle();
		int getDeltaPosition();
		// Get current velocity
		double getVelocity();
		// Get current acceleration
		double getAcceleration();
	};
} // namespace elib


#endif