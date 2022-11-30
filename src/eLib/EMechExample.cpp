#include <string>
using namespace std;

#include "eLib/Configurator.hpp"
#include "eLib/EMechanics.hpp"
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


// -----------------------------
// Mapping
// -----------------------------
#if USING_GPS && Arm_Installed_Rotation
Point Arm::calcGoalPosition(){
    if (attachedGoal != NULL){
        QAngle armAngle = angleBase + 1_deg * armRotation.getAngle();
        QLength newX = truePose->x + cos(truePose->theta) * (baseGoal2Center + cos(armAngle) * armLength);
        QLength newY = truePose->y + sin(truePose->theta) * (baseGoal2Center + cos(armAngle) * armLength);
        Point p(newX, newY);
        return p;
    }
    Point p(-1_m, -1_m);
    return p; 
}
void Arm::pickUpGoal(const string& goalName){
    actuateClamp();
    attachedGoal = TPField::pickUpGoal(goalName);
}
void Arm::dropGoal(){
    actuateClamp();
    TPField::dropMogo(attachedGoal, calcGoalPosition());
}
#endif














//----------------------------------------------
// 
// Rear Clamp Functions
// 
//----------------------------------------------
RearClamp::RearClamp(PistonConfig rClampCfg
            , PistonConfig rTilterCfg
            , OpticalConfig rOpticalCfg
            , AirTank * airTankPtr
            , shared_ptr<OdomState> rPose)

            : rearClampPiston(rClampCfg, airTankPtr)
            , rearTilterPiston(rTilterCfg, airTankPtr)
            #if Rear_Installed_Optical
            , rearOptical(rOpticalCfg)
            #endif
            , rearClampWatchTask(std::bind(&RearClamp::rearClampWatchFunc, this))
            {}

void RearClamp::rearClampWatchFunc(){
    while (true){
        if (doAction = true){
            // If Tilted, untilt and unclamp
            
            doAction = false;
        }
        pros::delay(10);
    }
}

// -----------------------------
// Motion
// -----------------------------
void RearClamp::actuateClamp(){rearClampPiston.actuatePiston();}
void RearClamp::setClamp(const bool& status){rearClampPiston.setPistonStatus(status);}
void RearClamp::actuateTilter(){rearTilterPiston.actuatePiston();}
void RearClamp::setTilter(const bool& status){rearTilterPiston.setPistonStatus(status);}


// -----------------------------
// Manual
// -----------------------------
// Notify watch task to 
void RearClamp::manualControl(){
    if(controller.get_digital_new_press(RearClampButton)){
        if (rearTilterPiston.getStatus() == Rear_Tilter_Up){
            setTilter(Rear_Tilter_Down);
            pros::delay(400);
            setClamp(Rear_Clamp_Up);
        }
        else{
            setClamp(Rear_Clamp_Down);
            pros::delay(100);
            setTilter(Rear_Tilter_Up);
        }
    }
}


// -----------------------------
// Status Monitor
// -----------------------------
#if Rear_Installed_Optical
string RearClamp::getMoGoal(){return rearOptical.getMoGoal();}
int RearClamp::getOpticalDist(){return rearOptical.getDist();}
double RearClamp::getOpticalHue(){return rearOptical.getHue();}
#endif


// -----------------------------
// Mapping
// -----------------------------
#if USING_GPS
Point RearClamp::calcGoalPosition(){
    if (attachedGoal != NULL){
        QLength newX = truePose->x + cos(truePose->theta) * goal2Center;
        QLength newY = truePose->y + sin(truePose->theta) * goal2Center;
        Point p(newX, newY);
        return p;
    }
    Point p(-1_m, -1_m);
    return p; 
}
void RearClamp::pickUpGoal(const string& goalName){
    actuateClamp();
    pros::delay(100);
    actuateTilter();
    attachedGoal = TPField::pickUpGoal(goalName);
}
void RearClamp::dropGoal(){
    actuateTilter();
    pros::delay(100);
    actuateClamp();
    TPField::dropMogo(attachedGoal, calcGoalPosition());
}
#endif










//----------------------------------------------
// 
//  Ring Mech Functions
// 
//----------------------------------------------
// Constructor
RingMech::RingMech(MotorConfig rm_cfg, Arm * armptr): 
                    ringMechMotor(ring_motor_cfg)
                    , ringWatchTask(std::bind(&RingMech::ringWatch, this))
                    {targetArm = armptr;}


// -----------------------------
// Watch Function
// -----------------------------

void RingMech::ringWatch(){
    Rate rateController;
    while(true){
        // If Ring Intake, do ring intake
        if (ringIntakeStatus == Ring_Intaking && !targetArm->isArmUpping()){
            ringMechMotor.setVoltage(Ring_Target_Voltage);
        }
        // Otherwise mirror whatever armMotor is doing
        else if(targetArm != NULL) ringMechMotor.setVoltage(-1 * targetArm->getVoltage());
        else{ringMechMotor.setVoltage(0);}
        rateController.delay(100_Hz);
    }
}



// -----------------------------
// Move
// -----------------------------

void RingMech::moveVoltage(int vol){
    if(vol){
        ringIntakeStatus = Ring_Intaking;
        ringMechMotor.setVoltage(vol);
    }
    else{
        ringIntakeStatus = Ring_Stopped;
    }
}



// -----------------------------
// Manual
// -----------------------------
void RingMech::manualControl(){
    // ringIntakeStatus = Ring_Intaking;
    if(controller.get_digital_new_press(RingMechIntakeButton)){
        if (ringIntakeStatus == Ring_Intaking) {ringIntakeStatus = Ring_Stopped;}
        else {ringIntakeStatus = Ring_Intaking;}
    }
}


// -----------------------------
// Monitor
// -----------------------------
double RingMech::getVoltage(){return ringMechMotor.getVoltage();}
