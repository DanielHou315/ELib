#ifndef E_DRIVE
#define E_DRIVE

#include <memory>
#include <list>
#include <fstream>
using namespace std;

// Configuraiton
#include "Configurator.hpp"
// Power
#include "eLib/eDriver/EDriver.hpp"
// SLAM
#include "eLib/eSLAM/EGPS.hpp"
// Planner
#include "eLib/eMotionPlanners/Searchers/EAStar.hpp"
#include "eLib/eMotionPlanners/Profilers/ELinearInterpolater.hpp"
// Controller
#include "eLib/eMotionControllers/EPid.hpp"
#include "eLib/eMotionControllers/EPursuit.hpp"
using namespace okapi;
using namespace elib;


enum DriveModes {E_DRIVE_MODE_MANUAL
                , E_DRIVE_MODE_PURSUIT
                , E_DRIVE_MODE_HOLD
                , E_DRIVE_MODE_TURN
                , E_DRIVE_MODE_SETTER
                };

enum DriveServices{E_DRIVE_TEMP_ACT_COMPLETE
                , E_DRIVE_TURN_IN_PURSUIT
                };

class EDrive{
private:
    //------------------------------------------------
    // Physical Construction
    //------------------------------------------------
    QLength driveWheelSeparation; 
    QLength driveWheelRadius;


    // -----------------------------
    // Power
    // -----------------------------
    // Motor
    EMotor LF,LM,LB,RF,RM,RB;
    double currentTargetPower[2];
    int lAveTemp, rAveTemp, lMaxTemp, rMaxTemp;

    // Transmission
    EPiston transmissionPiston;
    double gearRatios[2] = {0.36, 0.66};
    double * currentGearRatio = NULL;


    // -----------------------------
    // State Recorder
    // -----------------------------
    shared_ptr<OdomState> truePose;
    ofstream debugLogger;


    // -----------------------------
    // Planner
    // -----------------------------
    #if USING_GPS
    ELinearInterpolater pathProfiler;
    #endif

    // -----------------------------
    // Controllers
    // -----------------------------
    list<DriveModes> driveControllerQueue;
    pros::Task driveControl;
    pros::Mutex driveTaskMutex;

    // Manual
    // ESlewController manualSlewControllerL, manualSlewControllerR;

    // Hold Turn
    PidConfig holdPidConfig;
    PidConfig turnPidConfig;
    EMotorPid PidL, PidR;

    #if USING_GPS
    // Pursuit
    EPursuit pursuitCalculator;
    EMotorPid pursuitPid;
    int pursuitState;
    // dummies
    okapi::QLength p_len, p_tar;
    okapi::QAngle p_ang, t_ang;
    
    // Turn
    QAngle targetTurnAngle = 0_rad;
    #endif

    

    //------------------------------------------------
    // Functions
    //------------------------------------------------

    // -----------------------------
    // Drive
    // -----------------------------
    void driveMoveVoltage(const double& lpow, const double& rpow);


    // -----------------------------
    // Task 
    // -----------------------------
    void driveControlFunc();
    
    void prepHold(double targetLPos, double targerRPos);
    void prepTurn(QAngle t_ang);
    void prepPursuit(PathSegment targetPath, int _MaxPow, double _Slew, double initSpeed);
    void cleanUpPursuit();
    

    // -----------------------------
    // Cross-Mode Missions 
    // -----------------------------
    void doNextAct();
    void addAct(DriveModes mode);

    
public:

    EDrive(const MotorConfig& LTConfig
        , const MotorConfig& LMConfig
        , const MotorConfig& LBConfig
        , const MotorConfig& RTConfig
        , const MotorConfig& RMConfig
        , const MotorConfig& RBConfig
        , const PistonConfig& TransmitConfig
        , AirTank * airTankPtr
        , const PidConfig& pursuitParams
        , const PidConfig& holdarams
        , const PidConfig& turnParams
        , const QLength& driveLRSep
        , const QLength& dwheelD
        , shared_ptr<OdomState> odomSt);


    // -----------------------------
    // Manual Functions
    // -----------------------------
    void manualControl(const int &input_l, const int &input_r);
    void return2Manual();


    // -----------------------------
    // Transmission Shift
    // -----------------------------
    void shiftGear();
    void setGear(PistonStatus status);


    // -----------------------------
    // Auto Planning Functions
    // -----------------------------
    PathSegment makePath(vector<Point> pointSet, bool direction = false);


    // -----------------------------
    // Auto Motion Functions
    // -----------------------------
    void setVoltage(const int& lvol, const int& rvol);
    void hold(bool givenPosition = false);
    #if USING_GPS
    void turnAbsolute(const QAngle &rel_theta, int _MaxPow = 12000, double _Slew = 1000);
    void followPath(PathSegment targetPathtargetPath, int _MaxPow = 12000, double _Slew = 1000, double initSpeed = 0); 
    #endif 
    

    // -----------------------------
    // Wait Functions
    // -----------------------------
    #if USING_GPS
    void waitUntilPoint(const QLength& x_target, const QLength& y_target, const bool& exit, const QTime& maxWaitTime);
    void waitUntilActionEnd(const QTime& maxWaitTime);
    void waitUntilX(const QLength& absX, const QTime& maxWaitTime, const bool& exit);
    void waitUntilY(const QLength& absY, const QTime& maxWaitTime, const bool& exit);
    #endif


    // -----------------------------
    // Status Monitor & Updater
    // -----------------------------
    int getAutoDrive();
    string getAutoDriveStr();
    void getMotorPosition(double &lpos, double &rpos);
    void getVoltage(int &lpow, int &rpow);
    bool getGearSet();
};


#endif