#include "eLib/eDriver/Sensors/Rotation.hpp"
#include "eLib/utils/Units.hpp"
using namespace elib;

namespace elib{

    //----------------------------------------
	// ERotation Functions
	//----------------------------------------

	// Constructors
	ERotation::ERotation(const RotationConfig& rot_cfg)
		: rotationSensor(rot_cfg.port)
		#if ROTATION_CALCULATE
		, RotationTask(std::bind(&ERotation::pullValue, this))
		#endif
	{
		rotationSensor.set_reversed(rot_cfg.reversed);
		rotationSensor.reset_position();
	}

	// Set Position
	void ERotation::setPosition(int p){
		rotationSensor.set_position(p);
		#if ROTATION_CALCULATE
		currentPosition = p;
		prevPosition = p;
		#endif
		return;
	}
	void ERotation::setDataRate(int r){rotationSensor.set_data_rate(r);}


	// Get Readings
	int ERotation::getPosition(){return rotationSensor.get_position();}
	double ERotation::getAngle(){return rotationSensor.get_angle()/100.0;}


	#if ROTATION_CALCULATE
	// Pull Value
	void ERotation::pullValue()
	{
		okapi::Rate rateController;
		while(true){
			currentTimeStamp = pros::micros();

			// Calculate a time elapse
			deltaTime = (currentTimeStamp - prevTimeStamp) / 1000000;

			// Insert Current and Previous Encoder Reading
			currentPosition = rotationSensor.get_position();

			// Update Current and Previous Velocity
			prevVelocity = currentVelocity;
			currentVelocity = rotationSensor.get_velocity();

			// Update Current Acceleration
			Acceleration = (currentVelocity - prevVelocity) / deltaTime;

			// Update timestamp
			prevTimeStamp = currentTimeStamp;
			prevPosition = currentPosition;

			// Wait 10ms or less
			// pros::delay(10-(pros::millis()-prevTimeStamp)%10);
			rateController.delay(100_Hz);
		}
	}

	int ERotation::getDeltaPosition(){return currentPosition - prevPosition;}
	double ERotation::getVelocity(){return currentVelocity;}
	double ERotation::getAcceleration(){return Acceleration;}
	#endif
}