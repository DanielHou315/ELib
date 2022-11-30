#include "eLib/eDriver/Sensors/EOptical.hpp"
using namespace elib;

namespace elib{

    //----------------------------------------
	// Sensor Macros
	//----------------------------------------
	#define OpticalHueTwoStd 17
	#define OpticalBlue 232
	#define OpticalYellow 40
	#define OpticalRed 5

    //----------------------------------------
	// EOptical Functions
	//----------------------------------------

	EOptical::EOptical(const OpticalConfig& opt_cfg) : opticalSensor(opt_cfg.port) {opticalSensor.set_led_pwm(opt_cfg.led_pwm);}

	// Status Monitor
	int EOptical::getDist(){return opticalSensor.get_proximity();}
	double EOptical::getHue(){return opticalSensor.get_hue();}
	
	string EOptical::getColor(){
		int hue = getHue();
		if (hue < OpticalYellow+OpticalHueTwoStd && hue > OpticalYellow-OpticalHueTwoStd) return "Yellow";
		else if (hue < OpticalBlue+OpticalHueTwoStd && hue > OpticalBlue-OpticalHueTwoStd) return "Blue";
		else if (hue < OpticalRed+OpticalHueTwoStd || hue > 360+OpticalRed-OpticalHueTwoStd) return "Red";
		return "None";
	}

	// 2021-2022 Tipping Point Specific Function
	string EOptical::getMoGoal(){
		if (getColor() == "Blue" && getDist() >= 255) return "Blue Goal";
		else if (getColor() == "Yellow" && getDist() >= 255) return "Yellow Goal";
		else if (getColor() == "Red" && getDist() >= 255) return "Red Goal";
		return "None";
	}
    
}