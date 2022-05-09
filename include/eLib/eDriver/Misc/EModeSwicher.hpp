#ifndef E_MODE_SWITCHER
#define E_MODE_SWITCHER

#include "eLib/eDriver/Misc/EButton.hpp"
#include "eLib/eDriver/Misc/EAutoSelector.hpp"
#include "EScreenPrinter.hpp"
namespace elib{

    #define SelectAuto 0
    #define ChangeDebugDisplay 1
    #define DisplayImg 2
    #define DisplayMap 3

    class EModeSwitcher : public EBumperSwitch {
    private:
        pros::Task switcherMonitorTask;
        void switcherMonitor();

        // Members
        int selectorMode;
        const int maxModes = 2;
        
        // Daughter Memters
        EBumperButton button;

        EAutoSelector * autoSelector;
        EScreenPrinter * screenPrinter;

        void singlePressFunc();
        void longPressFunc();

    public:
        EModeSwitcher(char port
                    , EAutoSelector * autoSelPtr = NULL
                    , EScreenPrinter * scrPtr = NULL);
    };



} // namespace elib


#endif