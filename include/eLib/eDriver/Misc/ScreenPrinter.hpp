#ifndef E_SCREEN_PRINTER
#define E_SCREEN_PRINTER


#include "pros/rtos.hpp"

#include "eLib/DriveExample.hpp"
#include "eLib/MechExample.hpp"
#include "eLib/eDriver/Misc/AutoSelector.hpp"
#include "eLib/Configurator.hpp"
using namespace elib;

namespace elib{

    //------------------------------------
    // Print Stuff
    //------------------------------------
    /* Auto Print */
    #define AutoPrintLine 1
    #define DebugLine 2

    class EScreenPrinter{
    private:
        pros::Task screenPrint;
        void mainPrintFunc();

        pros::Mutex printMutex;
        int dbmode = 0;
        const int maxDebugMode = 4;
        
        int switcherMode = 0;
        bool shouldClear = false;

        EDrive * E_drive;
        Arm * E_arm;
        RearClamp * E_rearclamp;
        RingMech * E_ring;
        EAutoSelector * E_autoselector;
        shared_ptr<OdomState> truePose;

    public:
        EScreenPrinter(EDrive * driveptr
                    , Arm * fbptr
                    , RearClamp * rcptr
                    , RingMech * ringptr
                    , EAutoSelector * autoptr
                    , shared_ptr<OdomState> rPose);

        void printDebug();
        void changeDebug();

        void setSwitcherMode(int mode);
    };
}







#endif