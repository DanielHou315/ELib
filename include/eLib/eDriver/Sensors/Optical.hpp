#ifndef E_OPTICAL
#define E_OPTICAL

#include <string>
using namespace std;

#include "pros/optical.hpp"
#include "pros/rtos.hpp"



namespace elib{
    
    struct OpticalConfig{
        int port = 0;
        int led_pwm = 100;
		int detectionDistance;
        OpticalConfig(const int& p, int pwm = 100, int dist = 255){
			port = p;
			detectionDistance = dist;
			led_pwm = pwm;
		}
    };

    //--------------------------------
	// E Optical Sensor
	//--------------------------------
	class EOptical{
	// private:
	public: 
		pros::Optical opticalSensor;
		
	// public: 
		EOptical(const OpticalConfig& opt_cfg);

		// Status Monitor
		int getDist();
		double getHue();
		string getColor();

		// 2021-2022 Tipping Point Specific Functions
		string getMoGoal();
	};
} // namespace elib




#endif