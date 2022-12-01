#include <string>
using namespace std;

#include "pros/misc.hpp"

#include "eLib/DriveExample.hpp"
#include "eLib/utils/Math.hpp"
using namespace elib;
using namespace okapi;



//----------------------------------------
// Tank drive Functions
// 
// This example is a tank drive and its functions, 
// If you are using other drives, apply the abstract function returns to your physical drive model. 
//----------------------------------------




//--------------------------------------------------------------------------
//
// Constructor
//
//--------------------------------------------------------------------------
EDrive::EDrive(const MotorConfig& LFConfig
            , const MotorConfig& LMConfig
            , const MotorConfig& LBConfig
            , const MotorConfig& RFConfig
            , const MotorConfig& RMConfig
            , const MotorConfig& RBConfig
            , const PistonConfig& TransmitConfig
            , AirTank * airTankPtr
            , const PidConfig& pursuitParams
            , const PidConfig& holdParams
            , const PidConfig& turnParams
            , const QLength& drivedriveWheelSeparation
            , const QLength& dwheelD
            , shared_ptr<OdomState> odomSt)
            : LF(LFConfig) 
            , LM(LMConfig)
            , LB(LBConfig)
            , RF(RFConfig)
            , RM(RMConfig)
            , RB(RBConfig)
            , transmissionPiston(TransmitConfig, airTankPtr)
            , driveControl(std::bind(&EDrive::driveControlFunc, this))
            , holdPidConfig(holdParams)
            , turnPidConfig(turnParams)
            , PidL(holdPidConfig)
            , PidR(holdPidConfig)
            #if USING_GPS
            , pursuitPid(pursuitParams)
            , pursuitCalculator(odomSt)
            #endif
{
    truePose = odomSt;

    driveWheelSeparation = drivedriveWheelSeparation;
    driveWheelRadius = dwheelD / 2;

    currentGearRatio = & (gearRatios[transmissionPiston.getStatus()]);
}









//--------------------------------------------------------------------------
//
// Drive Basics
//
//--------------------------------------------------------------------------
// Give Power
void EDrive::driveMoveVoltage(const double& lpow, const double& rpow){
    currentTargetPower[0] = lpow;
    currentTargetPower[1] = rpow;
    LF.setVoltage(lpow);
    LM.setVoltage(lpow);
    LB.setVoltage(lpow);
    RF.setVoltage(rpow);
    RM.setVoltage(rpow);
    RB.setVoltage(rpow);
}

// A 2-speed Gearbox Transmission
void EDrive::shiftGear(){
    transmissionPiston.actuatePiston();
    if(transmissionPiston.getStatus() == E_DRIVE_360_RPM){EController::print(1, "Speed Drive");}
    else {EController::print(1, "Torque Drive");}
}
void EDrive::setGear(PistonStatus status){
    transmissionPiston.setPistonStatus(status);
    if (status == E_DRIVE_360_RPM) {EController::print(1, "Speed Drive");}
    else {EController::print(1, "Torque Drive");}
}










// -----------------------------
// Cross Mode Functions
// -----------------------------
void EDrive::doNextAct(){
    driveTaskMutex.take(20);
    if (driveControllerQueue.size() > 1){
        pros::lcd::print(7, "Cleaning Pursuit");
        if(driveControllerQueue.front() == E_DRIVE_MODE_PURSUIT){cleanUpPursuit();}
        pros::lcd::print(7, "Popping Pursuit");
        driveControllerQueue.pop_front();
    }
    pros::lcd::print(7, "Wrapping doNext");
    driveTaskMutex.give();
}
void EDrive::addAct(DriveModes mode){driveControllerQueue.push_back(mode);}









//--------------------------------------------------------------------------
//
// Drive Task Func
//
//--------------------------------------------------------------------------
// Task Watching the Drivetrain Status
void EDrive::driveControlFunc(){
    Rate rateController;                        // Control the Rate of Control
    while(true){
        // Temperature Monitor
        lAveTemp = (LF.getTempCelsius() + LM.getTempCelsius() + LB.getTempCelsius()) / 3;
        lMaxTemp = max(LF.getTempCelsius(), max(LM.getTempCelsius(), LB.getTempCelsius()));
        rAveTemp = (RF.getTempCelsius() + RM.getTempCelsius() + RB.getTempCelsius()) / 3;
        rMaxTemp = max(RF.getTempCelsius(), max(RM.getTempCelsius(), RB.getTempCelsius())); 


        // If Manual or Setter, do nothing
        if (driveControllerQueue.front() == E_DRIVE_MODE_HOLD){
            driveTaskMutex.take(20);
            double lpow = PidL.calculate(LM.getPosition());
            double rpow = PidR.calculate(RM.getPosition());
            driveMoveVoltage(lpow, rpow);
            driveTaskMutex.give();
        }
        #if USING_GPS
        // Pure Pursuit
        else if (driveControllerQueue.front() == E_DRIVE_MODE_PURSUIT){ 
            driveTaskMutex.take(20);
            // If the drive is in pursuit mode

            // Calculate the pursuit length and angle
            pursuitState = pursuitCalculator.calcPursuit(p_len, p_ang, p_tar);
            
            #if USING_TURN_IN_PURSUIT
            if (pursuitState){
                driveTaskMutex.give();
                doNextAct();
            }
            else{
            #endif
                // Difference in speed
                QLength targetR = p_tar / 2 / cos(M_PI/2-(p_ang).convert(radian));
                QLength targetRR = targetR + driveWheelSeparation/2;
                QLength targetLR = targetR - driveWheelSeparation/2;

                
                // Doesn't matter what unit it is, the PID parameters should be tuned accordingly
                // For the sake of consistency, I converted pursuit distance into motor rotations as error
                long double err = lin2rot(p_len, driveWheelRadius*2, *currentGearRatio).convert(degree);
                long double targetPow = pursuitPid.calculateFromError(err);

                // Apply Ratio
                long double lpow = targetPow * (targetLR.convert(meter) / targetR.convert(meter));
                long double rpow = targetPow * (targetRR.convert(meter) / targetR.convert(meter));

                
                if (abs(max(lpow,rpow)) > pursuitPid.getMax()){
                    long double rat = abs(max(lpow,rpow)) / pursuitPid.getMax();
                    lpow /= rat;
                    rpow /= rat;
                }

                ofstream debuggerLo;
                debuggerLo.open("/usd/Tuning/PID/DrivePursuit/drivePursuit.csv", std::ios_base::app);
                debuggerLo << NowMill << ',' << pursuitCalculator.findNearestIndex() << ',' << pursuitCalculator.targetIndex << ',' << pursuitCalculator.path.direction << ',' << p_len.convert(inch) << ',' << p_ang.convert(degree) << ',' << p_tar.convert(inch) << ',' << truePose->x.convert(inch) << ',' << truePose->y.convert(inch) << ',' << truePose->theta.convert(degree) << '\n';
                debuggerLo.close();


                driveMoveVoltage(lpow,rpow);
                driveTaskMutex.give();
            #if USING_TURN_IN_PURSUIT
            }
            #endif
            
        }
        else if (driveControllerQueue.front() == E_DRIVE_MODE_TURN){ 
            driveTaskMutex.take(20);
            // Get Difference in Angle
            QAngle delta_theta = getBoundAngle(targetTurnAngle - truePose->theta);
            QAngle lrot, rrot;
            // Set up left and right error angles
            turn2rot(delta_theta, driveWheelSeparation, driveWheelDiameter, *currentGearRatio, lrot, rrot);

            // pros::lcd::print(6, "LTarget %.2f RTarget %.2f",lrot.convert(degree), rrot.convert(degree));

            double lPow = PidL.calculateFromError(lrot.convert(degree));
            double rPow = PidR.calculateFromError(rrot.convert(degree));
            // Set Speed to motor
            driveMoveVoltage(lPow, rPow);
            
            ofstream debuggerLo;
            debuggerLo.open("/usd/Tuning/PID/DriveTurn/driveTurnPID.csv", std::ios_base::app);
            debuggerLo << truePose->theta.convert(degree) << ',' << lrot.convert(degree) << ',' << rrot.convert(degree) << ',' << lPow << ',' << rPow << '\n';
            debuggerLo.close();
            
            driveTaskMutex.give();
        }
        #endif
        rateController.delay(100_Hz);
    }
}

void EDrive::return2Manual(){
    driveControllerQueue.clear();
    addAct(E_DRIVE_MODE_MANUAL);
    doNextAct();
}


void EDrive::prepHold(double targetLPos, double targetRPos){
    PidL.setParameters(holdPidConfig);
    PidR.setParameters(holdPidConfig);
    PidL.setGoal(targetLPos);
    PidR.setGoal(targetRPos);
    PidL.setInitSpeed(12000);                 // Set hold Initial Speed Value to max so it can exert max force to hold at the very beginning
    PidL.setInitSpeed(12000);
}

void EDrive::prepTurn(QAngle t_ang){
    PidL.setParameters(turnPidConfig);
    PidR.setParameters(turnPidConfig);
    targetTurnAngle = t_ang;
    PidL.setInitSpeed(LM.getVoltage());                 // Set hold Initial Speed Value to max so it can exert max force to hold at the very beginning
    PidL.setInitSpeed(LM.getVoltage());
}

void EDrive::prepPursuit(PathSegment targetPath, int _MaxPow, double _Slew, double initSpeed){
    pursuitCalculator.startPath(targetPath);
    pursuitPid.setMax(_MaxPow);
    pursuitPid.setSlew(_Slew);
    pursuitPid.setInitSpeed(initSpeed);
}
void EDrive::cleanUpPursuit(){
    pursuitCalculator.clearPursuit();
    pursuitPid.setInitSpeed(0);
    pursuitPid.reset();
}

















//--------------------------------------------------------------------------
//
// Drive Functions
//
//--------------------------------------------------------------------------
// Parabolic Controller
void EDrive::manualControl(const int &input_l, const int &input_r){
    // If it currently allows manual drive, drive input
    if (driveControllerQueue.front() == E_DRIVE_MODE_MANUAL){

        // Slewed Drive Controller
        double lOut = input_l * abs(input_l) * DriveStickQuadraticCoeff;
        double rOut = input_r * abs(input_r) * DriveStickQuadraticCoeff;
        // driveMoveVoltage(manualSlewControllerL.limitWithSlew(lOut), manualSlewControllerR.limitWithSlew(rOut));
        driveMoveVoltage(lOut, rOut);
        // Gearbox Shifter
        if(controller.get_digital_new_press(TransmissionButton)) {
            shiftGear();
        }
    }
}








//--------------------------------------------------------------------------
//
// Auto Plan Functions
//
//--------------------------------------------------------------------------
PathSegment EDrive::makePath(vector<Point> pointSet, bool direction){
    PathSegment targetPath = pathProfiler.interpolatePath(pointSet, direction);
    return targetPath;
}



//--------------------------------------------------------------------------
//
// Move Functions
//
//--------------------------------------------------------------------------

void EDrive::setVoltage(const int& lvol, const int& rvol){
    if(driveControllerQueue.front() != E_DRIVE_MODE_SETTER){
        addAct(E_DRIVE_MODE_SETTER);
        doNextAct();
    }
    driveMoveVoltage(lvol, rvol);
}

// E_DRIVE_MODE_HOLD Drive in Current Position
void EDrive::hold(bool givenPosition){
    // If it has not been set to hold before, set to hold and set current positions. 
    if (getAutoDrive() != E_DRIVE_MODE_HOLD){
        pros::lcd::print(7, "Starting Prep");
        if (!givenPosition){prepHold(LM.getPosition(), RM.getPosition());}
        pros::lcd::print(7, "Adding Act");
        addAct(E_DRIVE_MODE_HOLD);
        pros::lcd::print(7, "Doing Act");
        doNextAct();
    }
}

#if USING_GPS
// E_DRIVE_MODE_TURN Based on Odometry Angle
void EDrive::turnAbsolute(const QAngle &abs_theta, int _MaxPow, double _Slew){
    driveTaskMutex.take(20);
    driveControllerQueue.clear();
    addAct(E_DRIVE_MODE_TURN);
    prepTurn(abs_theta);
    driveTaskMutex.give();
    doNextAct();
}

// Start async task Following the next path in Pure E_DRIVE_MODE_PURSUIT. 
void EDrive::followPath(PathSegment targetPath, int _MaxPow, double _Slew, double initSpeed){
    driveTaskMutex.take(20);
    driveControllerQueue.clear();
    addAct(E_DRIVE_MODE_PURSUIT);
    prepPursuit(targetPath, _MaxPow, _Slew, initSpeed);
    driveTaskMutex.give();
    doNextAct();
}
#endif





//--------------------------------------------------------------------------
//
// Wait Functions
//
//--------------------------------------------------------------------------

// Only return action complete when wait upon the end of action. 
void EDrive::waitUntilActionEnd(const QTime& maxWaitTime){
    int func_init_time = pros::millis();
    if(getAutoDrive() == E_DRIVE_MODE_PURSUIT){
        while(!pursuitCalculator.reachedPathEnd()){
            if (pros::millis() > func_init_time + maxWaitTime.convert(millisecond)){break;}
            pros::delay(10);
        }
    }
    else if (getAutoDrive() == E_DRIVE_MODE_TURN){
        while(abs(truePose->theta - targetTurnAngle) > 2_deg){
            if (pros::millis() > func_init_time + maxWaitTime.convert(millisecond)) break;
            pros::delay(10);
        }
    }
    if (driveControllerQueue.size() > 1) doNextAct();
    else hold();
    return;
}

#if USING_GPS
void EDrive::waitUntilPoint(const QLength& x_target, const QLength& y_target, const bool& exit, const QTime& maxWaitTime){
    QLength minDist = 9999_m;
    int increasingCnt = 0;

    int func_init_time = pros::millis();
    OdomState targetPose(x_target,y_target,0_rad);

    while(true){
        QLength tmpDist = distanceFromCurrentPose(truePose, targetPose);
        if (tmpDist < 0.5_in) break;                                                    // If point is awfully close, say it is next to it and exit. 
        if (tmpDist < minDist){                                                         // If the distance to point is decreasing, update minimum distance, wait
            minDist = tmpDist;
            increasingCnt = 0;
        }
        else{increasingCnt++;}                                                          // Otherwise cumulate number of iterations 

        if (increasingCnt > 3){break;}                                                  // If 3 continuous distance increase, break.
        if (pros::millis() > func_init_time+maxWaitTime.convert(millisecond)) break;    // If the time limit has reached, exit function.
        pros::delay(10);                                                                // Otherwise wait 10 ms
    }
    if (exit && driveControllerQueue.size() == 1) addAct(E_DRIVE_MODE_HOLD);
    if (driveControllerQueue.size() > 1) doNextAct();
    return;
}

void EDrive::waitUntilX(const QLength& _x, const QTime& maxWaitTime, const bool& exit){
    int func_init_time = pros::millis();
    while(abs(truePose->x - _x) > 1_in){
        if (pros::millis() > func_init_time + maxWaitTime.convert(millisecond))break;
        pros::delay(10);
    }
    if (exit && driveControllerQueue.size() == 1) addAct(E_DRIVE_MODE_HOLD);
    if (driveControllerQueue.size() > 1) doNextAct();
}
void EDrive::waitUntilY(const QLength& _y, const QTime& maxWaitTime, const bool& exit){
    int func_init_time = pros::millis();
    while(abs(truePose->y - _y) > 1_in){
        if (pros::millis() > func_init_time + maxWaitTime.convert(millisecond))break;
        pros::delay(10);
    }
    if (exit && driveControllerQueue.size() == 1) addAct(E_DRIVE_MODE_HOLD);
    if (driveControllerQueue.size() > 1) doNextAct();
}
#endif








//--------------------------------------------------------------------------
//
// Status Monitoring
//
//--------------------------------------------------------------------------

int EDrive::getAutoDrive(){return driveControllerQueue.front();}
string EDrive::getAutoDriveStr(){
    if (driveControllerQueue.front() == E_DRIVE_MODE_MANUAL) return "E_DRIVE_MODE_MANUAL";
    else if (driveControllerQueue.front() == E_DRIVE_MODE_SETTER) return "E_DRIVE_MODE_SETTER";
    #if USING_GPS
    else if (driveControllerQueue.front()  == E_DRIVE_MODE_PURSUIT) return "E_DRIVE_MODE_PURSUIT";
    else if (driveControllerQueue.front()  == E_DRIVE_MODE_TURN) return "E_DRIVE_MODE_TURN";
    #endif
    else if (driveControllerQueue.front()  == E_DRIVE_MODE_HOLD) return "E_DRIVE_MODE_HOLD";
    else return "NULL";
}


void EDrive::getMotorPosition(double &lpos, double &rpos){
    lpos = LB.getPosition();
    rpos = RB.getPosition();
}

void EDrive::getVoltage(int &lpow, int &rpow){
    lpow = LB.getVoltage();
    rpow = RB.getVoltage();
}

bool EDrive::getGearSet(){return transmissionPiston.getStatus();}