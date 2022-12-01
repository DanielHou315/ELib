#include "eLib/eDriver/Misc/Button.hpp"
#include "eLib/utils/Math.hpp"
using namespace elib;


namespace elib{
    EAbstractButton::EAbstractButton(){}

    EBumperButton::EBumperButton(char port) 
                                : EAbstractButton()
                                , bumperSwitch(port){}

    int EBumperButton::getCommand(){
        uint32_t now = NowMill;
        command = NO_PRESS;
        // If state changed...
        if(button_down != bumperSwitch.getStatus()){
            button_down = !button_down;
            if(button_down){button_down_ts = now;}                  // Timestamp button-down
            else{
                button_up_ts = now;                                 // Timestamp button-up
                if(now - button_down_ts > LONG_MILLIS_MIN){command = LONG_PRESS;}
                else {command = SINGLE_PRESS;}
            }
        }
        return command;
    }
}