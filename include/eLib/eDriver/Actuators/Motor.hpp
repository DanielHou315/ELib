#ifndef E_MOTOR
#define E_MOTOR

#include "pros/motors.hpp"
#include "pros/rtos.hpp"

#include "eLib/eMotionControllers/Pid.hpp"
using namespace elib;

#define Max_600_RPM pros::E_MOTOR_GEARSET_06
#define Max_200_RPM pros::E_MOTOR_GEARSET_18
#define Max_100_RPM pros::E_MOTOR_GEARSET_36

namespace elib{

    struct MotorConfig{
        int port;
        bool reversed = false;
        pros::motor_gearset_e_t gearset;
        pros::motor_brake_mode_e_t brake_mode;

        MotorConfig(const int &prt, const bool &rev,const pros::motor_gearset_e &grst, const pros::motor_brake_mode_e_t &brkmode){
            port = prt;
            reversed = rev;
            gearset = grst;
            brake_mode = brkmode;
        }
    };



	//----------------------------------------
	// E Motor
	//----------------------------------------
	class EMotor
	{
	// protected:
	public:
		pros::Motor emotor;
		// Output power - limit -12000~12000
		double outputVoltage = 0; 
		double maxVoltage = 12000;
		int loopTime = 0;

		pros::Mutex motor_power_mutex;

	// public:
		// Basic EMotor Constructor
		// @params:
		// - MotorConfig m_cfg
		EMotor(const MotorConfig& m_cfg);

		// set voltage
		// @params: 
		// - double Voltage (-12000 ~ 12000)
		virtual void setVoltage(const int &vol);

		// set max voltage
		// @params: 
		// - double Voltage (0 ~ 12000)
		virtual void setVoltageCap(int mxvol);

		// tare motor encoder position. 
		// @params: None
		virtual void tarePosition();

		// Status Monitoring

		// get position, in degrees
		// @params: None
		double getPosition();
		double getVoltage();
		int getTempCelsius();
		int getTempFahrenheit();
	};


	//----------------------------------------
	// E PID Motor
	//----------------------------------------
	class EPidMotor : public EMotor
	{
	protected:
		// Pid 
		bool m_enable;
		EMotorPid m_pid;
		pros::Task m_task;
		virtual void motorWatchFunc();

	public:
		// define port & pid tune
		EPidMotor(const MotorConfig& m_cfg, const PidConfig& pid_params);

		// Enable and Disable PID
		void enablePid();
		void disablePid();
		bool isEnabled();
		
		// Motion

		// set voltage
		// @params: 
		// - int Voltage (-12000 ~ 12000)
		void setVoltage(const int &vol);
		void setPidConfig(const PidConfig pid_params);
		void setTargetPosition(const double &desired);

		void waitUntilEncoderValue(int pos, int time);
		void waitUntil(int pos, int time);
		void hold(int additionalCmd = 0);

		// Status Monitor
		double getError();
	};
} // namespace elib

#endif

