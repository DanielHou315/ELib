#ifndef E_PISTON
#define E_PISTON

#include "pros/adi.hpp"
#include "pros/rtos.hpp"

#include "eLib/eDriver/Misc/Controller.hpp"
#include "eLib/eDriver/Misc/AirTank.hpp"
using namespace elib;


namespace elib{

    enum PistonType {E_PISTON_SINGLE_ACTION, E_PISTON_DOUBLE_ACTION};
    enum PistonStatus {E_PISTON_RETRACTED, E_PISTON_EXTENDED};
    
    struct PistonConfig{
        unsigned int port;
        unsigned int number_of_pistons = 1;
        PistonStatus initStatus;
        PistonType actionType;

        PistonConfig(const unsigned int &prt, const unsigned int& num,  const PistonStatus& init_stat, const PistonType& actype){
            port = prt;
            number_of_pistons - num;
            initStatus = init_stat;
            actionType = actype;
        }
    };

    //----------------------------------------
	// E Piston
	//----------------------------------------
	class EPiston
	{
	private:
		pros::ADIDigitalOut piston;
		PistonStatus pistonStatus;
		unsigned int pistonNumber;
		PistonType actionType;
		AirTank * air_tank;

	public:
		EPiston(const PistonConfig& piston_cfg, AirTank * AirtankPtr = NULL);
		void actuatePiston();
		void setPistonStatus(const bool& pStatus);
		PistonStatus getStatus();
	};

} // namespace elib

#endif

