#ifndef E_DRIVER_CONTROLLER
#define E_DRIVER_CONTROLLER

#include <string>
using namespace std;

#include "pros/misc.hpp"
#include "pros/rtos.hpp"

namespace elib{
    //----------------------------------------------
	//  Controller
	//----------------------------------------------
	// Functions that controls printing things on the LCD Screen of the Controller.
	static pros::Controller controller(pros::E_CONTROLLER_MASTER);

	class EController
	{
	private:
		pros::Task controllerPrinter;
		// Print
		static inline string LastPrints[2] = {"None0", "None1"};
		static inline string CurrentPrints[2] = {"None0", "None1"};

		// Rumble
		static inline string rumbleCmd = "";
		static inline int nextRumbleTime = 0;

		// Functions
		// Clear a line
		static void clearLine(int line);
		static void controllerPrinterFunc();
	public:
		EController();
		// Print a string in a line
		static void print(int line1, string str1);
		static void rumble(const char* rumbleCmd);
	};
}

#endif