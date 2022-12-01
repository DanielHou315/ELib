#include "pros/rtos.hpp"
#include "eLib/utils/Math.hpp"
#include "eLib/eDriver/Misc/Controller.hpp"
using namespace elib;

namespace elib{

    EController::EController()
                : controllerPrinter(controllerPrinterFunc)
                {}

    void EController::clearLine(int line){if (line < 2){controller.clear_line(line);}}
    void EController::controllerPrinterFunc(){
        static bool refreshing = false;
        while(true){
            refreshing = false;
            if(CurrentPrints[0] != LastPrints[0]){
                controller.clear_line(0);
                refreshing = true;
            }
            if(CurrentPrints[1] != LastPrints[1]){
                controller.clear_line(1);
                refreshing = true;
            }
            if (NowMill >= nextRumbleTime && rumbleCmd.size() && rumbleCmd.size() < 8){
                controller.rumble(rumbleCmd.c_str());
                rumbleCmd = "";
                nextRumbleTime = NowMill + 5000;
            }
            pros::delay(50);
            if (refreshing){
                controller.set_text(0,0,CurrentPrints[0]);
                controller.set_text(1,0,CurrentPrints[1]);
                LastPrints[0] = CurrentPrints[0];
                LastPrints[1] = CurrentPrints[1];
            }
        }
    }


    // Print a string in a line
    void EController::print(int line, string str){if (line < 2)CurrentPrints[line] = str;}

    void EController::rumble(const char* rumbleCmd){controller.rumble(rumbleCmd);}

}
    
    
    