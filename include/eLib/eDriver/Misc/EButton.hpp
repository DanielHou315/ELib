#ifndef E_BUTTON
#define E_BUTTON

#include <vector>
using namespace std;

#include "pros/rtos.hpp"

#include "eLib/utils/EUnits.hpp"
#include "eLib/eDriver/Sensors/EBumper.hpp"
using namespace elib;

namespace elib{
    enum EButtonEvent {
        NO_PRESS,
        SINGLE_PRESS,
        LONG_PRESS,
        DOUBLE_PRESS
    };

    class EAbstractButton{
    protected:
        const uint32_t LONG_MILLIS_MIN = 600;

        uint32_t button_down_ts = 0;
        uint32_t button_up_ts = 0;
        
        bool long_press_peding = false;
        bool button_down = false;

        EButtonEvent command;

    public:
        EAbstractButton();
    };


    
    class EBumperButton : EAbstractButton{
    private:
        EBumperSwitch bumperSwitch;
    public:
        EBumperButton(char port);
        int getCommand();
    };

    /*
    class EControllerButton{
    public:
        EControllerButton(pros::controller_analog_e_t conrollerButton);

    };
    */


}

#endif