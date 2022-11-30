#include "okapi/impl/util/rate.hpp"
using namespace okapi;

#include "eLib/eDriver/Misc/EModeSwicher.hpp"
#include "eLib/Configurator.hpp"
using namespace elib;

namespace elib{

    // Mode Switcher
    EModeSwitcher::EModeSwitcher(char port
                            , EAutoSelector * autoSelPtr
                            , EScreenPrinter * scrPtr)
                            : EBumperSwitch(port)
                            , switcherMonitorTask(std::bind(&EModeSwitcher::switcherMonitor, this))
                            , button(port)
                            {
                                autoSelector = autoSelPtr;
                                screenPrinter = scrPtr;
                            }

    void EModeSwitcher::switcherMonitor(){
        Rate rateController;
        while(true){
            int newCmd = button.getCommand();
            if (newCmd == LONG_PRESS) longPressFunc();
            else if (newCmd == SINGLE_PRESS) singlePressFunc();
            rateController.delay(50_Hz);
        }
    }

    //Single Press Function
    void EModeSwitcher::singlePressFunc(){      // Single Press to Advance to Next Option
        // Select Auto
        if(selectorMode == SelectAuto && autoSelector != NULL) autoSelector->changeAutoNumber();
        else if(selectorMode == ChangeDebugDisplay && screenPrinter != NULL) screenPrinter->changeDebug();
    }


    void EModeSwitcher::longPressFunc(){        // Long Press to Advance to Next Selection
        selectorMode++;
        if(selectorMode >= maxModes) selectorMode -= maxModes;
        screenPrinter->setSwitcherMode(selectorMode);
        return;
    }

}