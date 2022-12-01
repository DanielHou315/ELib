#include <string>
using namespace std;

#include "eLib/Configurator.hpp"
#include "eLib/MechExample.hpp"
using namespace elib;
using namespace okapi;



//----------------------------------------------
//  Arm Class Functions
//----------------------------------------------

Arm::Arm(MotorConfig fbMotorCfg 
        , PistonConfig fbClampCfg
        #if Arm_Installed_Cover
        , PistonConfig fbCoverCfg
        #endif
        #if Arm_Installed_HighPost
        , PistonConfig fbHighPostCfg
        #endif
        #if Arm_Installed_Optical
        , OpticalConfig fbOpticalCfg
        #endif
        #if Arm_Installed_Rotation
        , RotationConfig fbRotationCfg
        #endif
        , PidConfig fbMotorPidCfg
        , AirTank * airTankPtr
        , shared_ptr<OdomState> rPose) 

        : armMotor(fbMotorCfg, fbMotorPidCfg)
        , frontClampPiston(fbClampCfg, airTankPtr)
        #if Arm_Installed_Cover
        , frontMogoCover(fbCoverCfg, airTankPtr)
        #endif
        #if Arm_Installed_HighPost
        , highPostPiston(fbHighPostCfg, airTankPtr)
        #endif
        #if Arm_Installed_Optical
        , armOptical(fbOpticalCfg)
        #endif
        #if Arm_Installed_Rotation
        , armRotation(fbRotationCfg)
        #endif
        {
            armMotor.tarePosition();
            truePose = rPose;
        }


double Arm::highThres(){
    #if Arm_Installed_Rotation
    return tallestAngle;
    #else
    return lowThreshold;
    #endif
}
double Arm::lowThres(){
    #if Arm_Installed_Rotation
    return lowestAngle;
    #else
    return lowThreshold;
    #endif
}

// -----------------------------
// Manual
// -----------------------------
void Arm::manualControl(){
    // Arm Controller
    if(controller.get_digital(ArmUpButton) && getPosition() < highThres()) {
        armMoveVoltage(12000); 
        armUpping = true;
    }
    else if(controller.get_digital(ArmDownButton) && getPosition() > lowThres()) {
        armMoveVoltage(-10000);
        armUpping = false;
    }
    else if(getPosition() < highThres() + 5.0 && getPosition() > lowThres() - 1.0){
        armHold();
        armUpping = false;
    }
    else{
        armMoveVoltage(0);
        armUpping = false;
    }

    if(controller.get_digital_new_press(FrontClampButton)){actuateClamp();}

    #if Arm_Installed_Cover 
    if(controller.get_digital_new_press(FrontCoverButton)){actuateCover();} 
    #endif
}


// -----------------------------
// Movement
// -----------------------------
void Arm::armHold(){
    if(!armMotor.isEnabled()){
        armMotor.hold(1);
        armMotor.enablePid();
    }
}
void Arm::armMoveVoltage(const int & voltage){
    armMotor.setVoltage(voltage);
    armMotor.disablePid();
}
void Arm::armMoveTo(const double& position){
    int targetPos = armMotor.getPosition() 
                    + (max(min(highThres(), position), lowThres()) 
                            - armRotation.getAngle())
                    * 3.5;
    armMotor.setTargetPosition(targetPos);
    armMotor.enablePid();
}


// -----------------------------
// Pistons
// -----------------------------
void Arm::actuateClamp(){frontClampPiston.actuatePiston();}
void Arm::setClamp(const bool& status){frontClampPiston.setPistonStatus(status);}
#if Arm_Installed_Cover
void Arm::actuateCover(){frontMogoCover.actuatePiston();}
void Arm::setCover(const bool& status){frontMogoCover.setPistonStatus(status);}
#endif
#if Arm_Installed_HighPost
void Arm::actuateHighPost(){highPostPiston.actuatePiston();}
void Arm::setHighPost(const bool& status){highPostPiston.setPistonStatus(status);}
#endif


// -----------------------------
// Wait
// -----------------------------
void Arm::waitUntilError(const int& err, const QTime& time){armMotor.waitUntil(err, time.convert(millisecond));}
void Arm::waitUntilPosition(const double& position, const QTime& time){while(abs(getPosition() - position) > 50) pros::delay(5);}


// -----------------------------
// Status Monitor
// -----------------------------
double Arm::getPosition(){
    #if Arm_Installed_Rotation
    return armRotation.getAngle();
    #else
    return armMotor.getPosition();
    #endif
}
bool Arm::isEnabled(){return armMotor.isEnabled();}
double Arm::getVoltage(){return armMotor.getVoltage();}
bool Arm::isArmUpping(){return armUpping;}
#if Arm_Installed_Optical
int Arm::getOpticalDist(){return armOptical.getDist();}
string Arm::getMoGoal(){return armOptical.getMoGoal();}
#endif