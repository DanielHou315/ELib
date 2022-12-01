#include "eLib/eDriver/Actuators/Motor.hpp"
#include "eLib/utils/Math.hpp"
using namespace elib;

namespace elib{

    EMotor::EMotor(const MotorConfig& m_cfg) 
            : emotor(m_cfg.port
            , m_cfg.gearset
            , m_cfg.reversed
            , pros::E_MOTOR_ENCODER_DEGREES)
            {emotor.set_brake_mode(m_cfg.brake_mode);}

    // Set Power Externally 
    void EMotor::setVoltage(const int &vol){
        motor_power_mutex.take(10);
        emotor.move_voltage(vol);
        outputVoltage = vol;
        motor_power_mutex.give();
    }
    void EMotor::setVoltageCap(int mxvol){
        motor_power_mutex.take(10);
        maxVoltage = min(abs(mxvol), 12000);
        motor_power_mutex.give();
    }
    void EMotor::tarePosition(){emotor.tare_position();}
    // Get Current Position
    double EMotor::getPosition(){return emotor.get_position();}
    double EMotor::getVoltage(){return outputVoltage;}
    int EMotor::getTempCelsius(){return emotor.get_temperature();}
	int EMotor::getTempFahrenheit(){return (int)emotor.get_temperature()*1.8+32;}





    //----------------------------------------------
    //  E PID Motor Functions
    //----------------------------------------------

    EPidMotor::EPidMotor(const MotorConfig& m_cfg, const PidConfig& pid_params) : 
                    EMotor(m_cfg), m_pid(pid_params),
                    m_task(std::bind(&EPidMotor::motorWatchFunc, this)){}

    // Watch PID
    void EPidMotor::motorWatchFunc(){
        okapi::Rate motorRate;
        while (true){
            loopTime = pros::millis();
            motor_power_mutex.take(10);
            if (m_enable){
                outputVoltage = m_pid.calculate(getPosition());
            }
            emotor.move_voltage(outputVoltage);
            motor_power_mutex.give();
            motorRate.delay(100_Hz);
        }
    }

    // Enable and Disable PID
    void EPidMotor::enablePid() {m_enable = true;}
    void EPidMotor::disablePid() {m_enable = false;}
    bool EPidMotor::isEnabled(){return m_enable;}

    void EPidMotor::setPidConfig(const PidConfig pid_params){
        motor_power_mutex.take();
        m_pid.setParameters(pid_params);
        motor_power_mutex.give();
    }
    void EPidMotor::setTargetPosition(const double &desired){
        motor_power_mutex.take(10);
        m_pid.setGoal(desired);
        motor_power_mutex.give();
    }
    void EPidMotor::setVoltage(const int &vol){
        disablePid();
        motor_power_mutex.take(10);
        outputVoltage = vol;
        motor_power_mutex.give();
    }
    void EPidMotor::hold(int additionalCmd){setTargetPosition(getPosition());}
    void EPidMotor::waitUntil(int err, int time){m_pid.waitUntilError(err, time);}
    void EPidMotor::waitUntilEncoderValue(int val, int time){m_pid.waitUntilError(val-m_pid.getGoal(),time);}

    double EPidMotor::getError(){return m_pid.getError();}
}