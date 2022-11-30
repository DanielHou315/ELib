#ifndef ETP21_MECH
#define ETP21_MECH

#include "eLib/eDriver/EDriver.hpp"
#include "eLib/utils/EMath.hpp"
#include "eLib/eSLAM/EGPS.hpp"
#include "eLib/Configurator.hpp"
using namespace elib;

#if USING_GPS
#include "eGame/ETPGame.hpp"
using namespace etp;
#endif










//----------------------------------------------
//  Four Bar
//----------------------------------------------
class Arm{
private:
    // Variables
    double armPos;
    bool armUpping = false;
    #if Arm_Installed_Rotation
    const double lowestAngle = 1.5;
    const double tallestAngle = 100.0;
    const double motor2AngleRatio = 3.5;
    #else 
    const double highThreshold = 3750.0;
    const double lowThreshold = -200.0;
    #endif
    

    // Power
    #if Arm_Installed_Motor
    EPidMotor armMotor;
    #endif

    #if Arm_Installed_Clamp
    EPiston frontClampPiston;
    #endif

    #if Arm_Installed_Cover
    EPiston frontMogoCover;
    #endif

    #if Arm_Installed_HighPost
    EPiston highPostPiston;
    #endif

    // Sensors
    #if Arm_Installed_Optical
    EOptical armOptical;
    #endif
    #if Arm_Installed_Rotation
    ERotation armRotation;
    #endif

    // Pneumatics
    AirTank * airTank;

    #if USING_GPS && Arm_Installed_Rotation
    const QAngle angleBase = 135_deg;
    const QLength baseGoal2Center = 7_in, armLength = 30_in;
    shared_ptr<OdomState> truePose;
    MobileGoal * attachedGoal = NULL;
    #endif

    double highThres();
    double lowThres();
    
public: 
    Arm(MotorConfig fbMotorCfg 
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
            , shared_ptr<OdomState> rPose);
    
    // -----------------------------
    // Movement
    // -----------------------------
    // Motors
    void armMoveVoltage(const int& voltage);
    void armHold();
    void armMoveTo(const double& position);

    // Pistons
    void actuateClamp();
    void setClamp(const bool& status);
    #if Arm_Installed_Cover
    void actuateCover();
    void setCover(const bool& status);
    #endif
    #if Arm_Installed_HighPost
    void actuateHighPost();
    void setHighPost(const bool& status);
    #endif


    // -----------------------------
    // Wait
    // -----------------------------
    void waitUntilError(const int& err, const QTime & time);
    void waitUntilPosition(const double& position, const QTime & time);
    

    // -----------------------------
    // Manual
    // -----------------------------
    void manualControl();


    // -----------------------------
    // Status Monitoring
    // -----------------------------
    double getPosition();
    double getVoltage();
    bool isEnabled();
    bool isArmUpping();
    #if Arm_Installed_Optical
    int getOpticalDist();
    string getMoGoal();
    #endif

    // -----------------------------
    // Mapping
    // -----------------------------
    #if USING_GPS
    Point calcGoalPosition();
    void pickUpGoal(const string& goalName);
    void dropGoal();
    #endif
};










//----------------------------------------------
//  Rear Clamp
//----------------------------------------------
class RearClamp{
private:
    EPiston rearClampPiston;
    EPiston rearTilterPiston;

    #if Rear_Installed_Optical
    EOptical rearOptical;
    #endif

    // Pneumatics
    AirTank * airTank;

    // Task
    pros::Task rearClampWatchTask;
    bool doAction = false;
    

    #if USING_GPS
    // Mapping Related
    const QLength goal2Center = -11_in;
    shared_ptr<OdomState> truePose;
    MobileGoal * attachedGoal = NULL;
    #endif

public: 
    RearClamp(PistonConfig rClampCfg
            , PistonConfig rTilterCfg
            , OpticalConfig rOpticalCfg
            , AirTank * airTankPtr
            , shared_ptr<OdomState> rPose);

    void rearClampWatchFunc();
    

    // -----------------------------
    // Motion
    // -----------------------------
    void actuateClamp();
    void setClamp(const bool& status);
    void actuateTilter();
    void setTilter(const bool& status);


    // -----------------------------
    // Manual
    // -----------------------------
    void manualControl();

    // -----------------------------
    // Status Monitor
    // -----------------------------
    #if Rear_Installed_Optical
    string getMoGoal();
    int getOpticalDist();
    double getOpticalHue();
    #endif

    // -----------------------------
    // Mapping
    // -----------------------------
    #if USING_GPS
    Point calcGoalPosition();
    void pickUpGoal(const string& goalName);
    void dropGoal();
    #endif
};









//----------------------------------------------
//  Ring Mech
//----------------------------------------------
// Macros for Rings
#define Ring_Intaking 1
#define Ring_Stopped 0
#define Ring_Target_Voltage 12000

class RingMech{
private:
    EMotor ringMechMotor;
    // Status Monitor
    bool ringIntakeStatus = true;

    Arm * targetArm;
    pros::Task ringWatchTask;
    void ringWatch();
    
public: 
    RingMech(MotorConfig rm_cfg, Arm * arm = NULL);
    void moveVoltage(int voltage);
    void manualControl();

    double getVoltage();
};


#endif