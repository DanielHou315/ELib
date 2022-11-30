#include "eLib/eMotionControllers/EPid.hpp"
#include "eLib/utils/EMath.hpp"
using namespace elib;
using namespace okapi;

namespace elib{
    //----------------------------------------------
    // EMotor PID Functions
    //----------------------------------------------

    EBasePid::EBasePid(const PidConfig& pid_params){
        kP = pid_params._kP;   
        kI = pid_params._kI;
        kD = pid_params._kD;
        kF = pid_params._kF;
        IntegralMax = pid_params._IntegralMax;
        IntegralRange = pid_params._IntegralRange;

        goalMax = pid_params._goalMax;
        slewRate = pid_params._slew;
    }

    void EBasePid::setParameters(const PidConfig& pid_params){
        kP = pid_params._kP;   
        kI = pid_params._kI;
        kD = pid_params._kD;
        kF = pid_params._kF;
        IntegralMax = pid_params._IntegralMax;
        IntegralRange = pid_params._IntegralRange;

        goalMax = pid_params._goalMax;
        slewRate = pid_params._slew;
    }

    // Set PID goal
    void EBasePid::setGoal(double goal){desired = goal;}
    // Set Max Motor Move Value
    void EBasePid::setMax(double maxMotorValue){goalMax = maxMotorValue;}
    // set slew
    void EBasePid::setSlew(double r){slewRate = r;}
    // Set Initial Speed required by slew controller
    void EBasePid::setInitSpeed(const double& pow){lastError = pow;}

    // Status Monitor
    double EBasePid::getError(){return error;}
    double EBasePid::getGoal(){return desired;}
    double EBasePid::getMax(){return goalMax;}
	double EBasePid::getSlew(){return slewRate;}


    void EBasePid::reset(){
        lastError = 0;
        lastOutput = 0;
        Integral = 0;
        desired = 0;
        error = 0;
    }




    EMotorPid::EMotorPid(const PidConfig& pid_params) : EBasePid(pid_params){
        kP = pid_params._kP; // Set Constants
        kI = pid_params._kI;
        kD = pid_params._kD;
        kF = pid_params._kF;
        Integral = 0.0; // Reset Integral
        IntegralMax = pid_params._IntegralMax;
        IntegralRange = pid_params._IntegralRange;
        lastError = 0.0; // Last Error
    }

    double EMotorPid::calculate(double measuredInput){
        // Calculate Error
        error = desired - measuredInput;
        return calculateFromError(error);
    }

    double EMotorPid::calculateFromError(double _error){
        error = _error;
        // Only calculate I (error sum) within a certain range
        if (fabs(error) < IntegralRange){Integral += error;}

        // Capping I
        // If Ki*Integral outputs a value greater than the max motor value
        // Then cap Integral to be the max Integral we define (max intagral)

        Integral = max(min(Integral, IntegralMax / kI), -IntegralMax / kI);
        // if ((Integral * kI) > IntegralMax){Integral = IntegralMax / kI;}
        // else if ((Integral * kI) < -IntegralMax){Integral = -IntegralMax / kI;}

        // Set Integral to 0.0 at crossing.
        if (sgn(error) != sgn(lastError)){Integral = 0.0;}

        // Calulate the output... P + I + D + F
        double output = (error * kP) + (Integral * kI) + (error - lastError) * kD + (desired * kF);

        // Control max speed and slew
        // Control max speed
        // If the target max speed is less than the old max speed, add slew until max speed is reached
        
        
        if (goalMax > lastOutput){lastOutput += slewRate;}
        else if (goalMax < lastOutput){lastOutput = goalMax;}

        // Cap the output so that it does not output more than abs(max)
        output = output > lastOutput?lastOutput:output;
        output = output < (-lastOutput)?(-lastOutput):output;
        

        // Store the previous error
        lastError = error;

        // Returns output
        return output;
    }

    void EMotorPid::waitUntilError(double _error, double time){
        int i = 0;
        while (true)
        {
            if (fabs(error - _error) > 50) {i = 0;}
            else{i++;}
            pros::delay(1); // Small delay for fast reaction. Helps overall precision.
            if (i > time){break;}
        }
    }

} // namespace elib