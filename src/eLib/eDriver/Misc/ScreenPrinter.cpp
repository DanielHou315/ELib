#include "pros/llemu.hpp"
#include "display/lvgl.h"

#include "eLib/Configurator.hpp"
#include "eLib/eDriver/Misc/ScreenPrinter.hpp"
using namespace elib;



namespace elib{

    EScreenPrinter::EScreenPrinter(EDrive * driveptr
                                , Arm * fbptr
                                , RearClamp * rcptr
                                , RingMech * ringptr
                                , EAutoSelector * autoptr
                                , shared_ptr<OdomState> rPose)
    : screenPrint(std::bind(&EScreenPrinter::mainPrintFunc, this))
    {
        E_drive = driveptr;
        E_arm = fbptr;
        E_rearclamp = rcptr;
        E_ring = ringptr;
        E_autoselector = autoptr;
        truePose = rPose;
    }

    void EScreenPrinter::changeDebug(){
        dbmode++;
        if(dbmode >= maxDebugMode){dbmode -= maxDebugMode;}
    }



    void EScreenPrinter::mainPrintFunc(){
        Rate rateController;
        static int lnt, rnt;
        while(true){
            // System Status
            if(switcherMode == 0) pros::lcd::print(0, "Select Auto");
            else pros::lcd::print(0, "Debug : %d", dbmode);
            
            if(E_autoselector->getAutoNumber() == 0) pros::lcd::print(1, "AWP");
            else if(E_autoselector->getAutoNumber() == 1) pros::lcd::print(1, "AWP_Mid");
            else if(E_autoselector->getAutoNumber() == 2) pros::lcd::print(1, "Left_Half_NC");
            else if(E_autoselector->getAutoNumber() == 3) pros::lcd::print(1, "Left_Half_Cover");
            else if(E_autoselector->getAutoNumber() == 4) pros::lcd::print(1, "Right_Half_NC");
            else if(E_autoselector->getAutoNumber() == 5) pros::lcd::print(1, "Right_Half_Cover");
            else if(E_autoselector->getAutoNumber() == 6) pros::lcd::print(1, "MidRush_NC");
            else if(E_autoselector->getAutoNumber() == 7) pros::lcd::print(1, "MidRush_Cover");
            else if(E_autoselector->getAutoNumber() == 8) pros::lcd::print(1, "MidRush_Fakeout");
            else if(E_autoselector->getAutoNumber() == 9) pros::lcd::print(1, "Left WP Only");

            if (dbmode == 0){
                pros::lcd::print(DebugLine, "AutoDrive: %d",E_drive->getAutoDrive());
                E_drive->getVoltage(lnt, rnt);
                pros::lcd::print(DebugLine+1, "LPow: %d, RPow: %d", lnt, rnt);
                #if USING_GPS
                pros::lcd::print(DebugLine+2, "X: %.2f, Y: %.2f, Yaw: %.2f", truePose->x.convert(inch), truePose->y.convert(inch), truePose->theta.convert(degree));
                #else
                pros::lcd::clear_line(DebugLine+2);
                #endif
            }
            else if (dbmode == 1){
                pros::lcd::print(DebugLine, "FB Pos: %.2f Pow: %.2f", E_arm->getPosition(), E_arm->getVoltage());
                #if Arm_Installed_Optical
                pros::lcd::print(DebugLine+1, "FB Dist: %d", E_arm->getOpticalDist());
                pros::lcd::print(DebugLine+2, "FB Mogo: %s", E_arm->getMoGoal());
                #else 
                pros::lcd::clear_line(DebugLine+1);
                pros::lcd::clear_line(DebugLine+2);
                #endif
            }
            else if (dbmode == 2){
                #if Rear_Installed_Optical
                pros::lcd::print(DebugLine, "RC Dist: %d", E_rearclamp->getOpticalDist());
                pros::lcd::print(DebugLine+1, "RC Hue: %.2f", E_rearclamp->getOpticalHue());
                pros::lcd::print(DebugLine+2, "RC MoGoal: %s", E_rearclamp->getMoGoal());
                #else
                pros::lcd::clear_line(DebugLine);
                pros::lcd::clear_line(DebugLine+1);
                pros::lcd::clear_line(DebugLine+2);
                #endif
            }
            else if (dbmode == 3){
                pros::lcd::print(DebugLine, "Ring Voltage %.2f", E_ring->getVoltage());
                pros::lcd::clear_line(DebugLine+1);
                pros::lcd::clear_line(DebugLine+2);
            }
            rateController.delay(50_Hz);
        }
    }


    void EScreenPrinter::setSwitcherMode(int mode){switcherMode = mode;}
}