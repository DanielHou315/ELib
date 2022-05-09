#ifndef E_BUMPER_SWITCH
#define E_BUMPER_SWITCH

#include <vector>
using namespace std;

#include "pros/adi.hpp"
#include "pros/rtos.hpp"

#include "eLib/eDriver/Misc/EAutoSelector.hpp"



namespace elib{

    #define BUMPER_ACTUATED 1
    #define BUMPER_RELEASED 0

    class EBumperSwitch{
    protected:
        pros::ADIDigitalIn bumperSwitch;
    public:
        EBumperSwitch(char port);         // isBareBumper determines if it is initialized as part of other class or as a bare bumper
        bool getStatus();
    };

} // namespace elib


#endif