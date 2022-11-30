#ifndef E_AIRTANK
#define E_AIRTANK

#include <string>
using namespace std;

#include "pros/misc.hpp"
#include "pros/rtos.hpp"

namespace elib{
    //----------------------------------------------
	//  Air Tank
	//----------------------------------------------
	class AirTank
	{
	private:
		// air left in airtank
		double CurPSI;
		const short MaxPSIByLaw = 100;
		pros::Task airtankwatcher;
		
		bool lowAirWarned = false;

	public:
		// Constructor
		AirTank();
		void CheckPSI();
		
		// Record an activation of a piston
		void AddActuation(int num = 1);
		// Print PSI on the Controller LCD
		int GetPSI();
	};
}

#endif