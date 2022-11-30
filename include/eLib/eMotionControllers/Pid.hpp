#ifndef E_DRIVE_PID
#define E_DRIVE_PID

#include "pros/rtos.hpp"

namespace elib{

	struct PidConfig{
        double _kP = 0, _kI = 0, _kD = 0, _kF = 0;
        double _IntegralRange = 0;
        double _IntegralMax = 0;
        int _goalMax = 12000; 		// Max motor power
		double _slew = 100;

        PidConfig(const double& p,const double& i,const double& d,const double& f,
                const double& integran, const double& integmax,
                const double& goalmaxi = 12000, const double& slew = 100){
            _kP = p;
            _kI = i;
            _kD = d;
            _kF = f;
            _IntegralRange = integran;
            _IntegralMax = integmax;
            _goalMax = goalmaxi;
			_slew = slew;
        }
    };


	class EBasePid{
	protected:
		double kP = 0, kI = 0, kD = 0, kF = 0;
		double Integral = 0;
		double lastError = 0;
		double IntegralRange = 0;
		double IntegralMax = 0;
		double desired = 0;
		double error = 0;

		double lastOutput = 0;
		double goalMax = 0;
		double slewRate = 0;

	public:
		EBasePid(const PidConfig& pid_params);

		void setParameters(const PidConfig& pid_params);
		// Set PID goal
		void setGoal(double goal);
		// Set Max Motor Move Value
		void setMax(double maxMotorValue);
		// set slew
		void setSlew(double startPower);
		// Set Initial Speed required by slew controller
		void setInitSpeed(const double& pow);

		// Status Monitor
		double getError();		
		double getGoal();
		double getMax();
		double getSlew();

		void reset();

		virtual double calculate(double processVariable) = 0;
		virtual double calculateFromError(double _error) = 0;
		virtual void waitUntilError(double _error, double time) = 0;
	};


	//--------------------------------
	// Single Motor PID
	//--------------------------------
	class EMotorPid : public EBasePid
	{
	public:
		EMotorPid(const PidConfig& pid_params);

		virtual double calculate(double processVariable);
		virtual double calculateFromError(double error);
		virtual void waitUntilError(double _error, double time);
	};
}


#endif