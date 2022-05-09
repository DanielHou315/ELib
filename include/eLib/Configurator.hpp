#ifndef _ETP20_CONFIGURATOR_
#define _ETP20_CONFIGURATOR_

#include "eLib/utils/EUnits.hpp"
using namespace okapi;

#include "eLib/eDriver/EDriver.hpp"
#include "eLib/eDriver/Actuators/EPiston.hpp"
#include "eLib/eSLAM/localization/EOdometry.hpp"
using namespace elib;


//---------------------------------------
// 
// PROGRAM SETUP
// 
// -- Whether Stable Release
// -- Debug Mode Tester
//---------------------------------------
#define STABLE_RELEASE true


#define USING_SHIFT_BUTTON true

// Control
#define LeftDriveStick ANALOG_LEFT_Y
#define RightDriveStick ANALOG_RIGHT_Y

#define TransmissionButton pros::E_CONTROLLER_DIGITAL_Y

#define FrontClampButton pros::E_CONTROLLER_DIGITAL_L1
#define FrontCoverButton pros::E_CONTROLLER_DIGITAL_RIGHT

#define RearClampButton pros::E_CONTROLLER_DIGITAL_L2

#define ArmUpButton pros::E_CONTROLLER_DIGITAL_R1
#define ArmDownButton pros::E_CONTROLLER_DIGITAL_R2

#define RingMechIntakeButton pros::E_CONTROLLER_DIGITAL_B

//---------------------------------------
//
// General Motor Macros
//
//---------------------------------------
#define motor_brake_coast pros::E_MOTOR_BRAKE_COAST
#define motor_gearset_100 pros::E_MOTOR_GEARSET_36
#define motor_gearset_200 pros::E_MOTOR_GEARSET_18
#define motor_gearset_600 pros::E_MOTOR_GEARSET_06




//---------------------------------------
// 
// Mode Switcher Parameters
// 
//---------------------------------------
#define ModeSwitcherButtonPort 'f'
#define MAX_AUTO_NUMBER 10








//---------------------------------------
// 
// DriveTrain Parameters
// 
//---------------------------------------
// Compiler Def
#define USING_GPS true
#define USING_TURN_IN_PURSUIT false




#define DriveStickQuadraticCoeff 0.732421875  //    X / 128 (normalize square) / 128 (normalize by move() function) * 12000 (scale to move_voltage())
#define E_DRIVE_360_RPM E_PISTON_EXTENDED 
#define E_DRIVE_216_RPM E_PISTON_RETRACTED


// Motors
static const MotorConfig driveLF_cfg(20, false, motor_gearset_600, motor_brake_coast);
static const MotorConfig driveLM_cfg(18, true, motor_gearset_600, motor_brake_coast);
static const MotorConfig driveLB_cfg(17, false, motor_gearset_600, motor_brake_coast);
static const MotorConfig driveRF_cfg(11, true, motor_gearset_600, motor_brake_coast);
static const MotorConfig driveRM_cfg(12, false, motor_gearset_600, motor_brake_coast);
static const MotorConfig driveRB_cfg(14, true, motor_gearset_600, motor_brake_coast);

static PistonConfig drive_transmission_piston_cfg('h',2, E_DRIVE_360_RPM, E_PISTON_DOUBLE_ACTION);

// Drivetrain Dimentions
// Separation between the CENTER of the Left and Right solid rubber wheels. 
static const QLength driveWheelSeparation = 13_in;
// The radius of the solid rubber wheel on the drivetrain
static const QLength driveWheelDiameter = 3.25_in;

//---------------------------------------
// PID 
//---------------------------------------
// static PidConfig drivePursuitPidConfig(20,300,95,0,7200,0.25);
static PidConfig drivePursuitPidConfig(40,  3,80,0,100,1400);
static PidConfig driveTurnPidConfig(   35,  6,100,0,50,1000,12000,2000);
static PidConfig driveHoldPidConfig(   100,0,0,0,0,0);











//---------------------------------------
// 
// GPS Parameters
// 
//---------------------------------------
#if USING_GPS

#define USING_PATH_FINDING true

// IMUs
// Top IMU
static const IMUConfig gps_imu1_config(3, 1, 0, 2, true, true, true);
// Bottom IMU
static const IMUConfig gps_imu2_config(8, 1, 0, 2, false, true, false);

// Tracking Wheels
static const TrackingWheelConfig ltrkwhlconfig(6,false, 2.75_in);
static const TrackingWheelConfig rtrkwhlconfig(5, true, 2.75_in);

// Odom Chassis Config
static const OdomChassisConfig chassisCfg(7_in,11_in,8.5_in,8.5_in,3_in,3_in);
// Initialize a Pose
static const OdomState initPose(0_in, 0_in, 90_deg);
#endif










//---------------------------------------
// Arm
//---------------------------------------
// Compiler def
#define Arm_Installed_Motor true
#define Arm_Installed_Clamp true
#define Arm_Installed_Rotation true
#define Arm_Installed_Optical true
#define Arm_Installed_Cover true
#define Arm_Installed_HighPost false


// Power
#if Arm_Installed_Motor
static MotorConfig arm_motor_cfg(1, true, motor_gearset_100, motor_brake_coast);
static PidConfig arm_motor_pid_cfg(800,4,0,0,40,2000);
#endif

// Clamp
#if Arm_Installed_Clamp
#define Arm_Clamp_Down E_PISTON_RETRACTED
#define Arm_Clamp_Up E_PISTON_EXTENDED
static PistonConfig arm_clamp_piston_cfg('a',1,E_PISTON_RETRACTED, E_PISTON_DOUBLE_ACTION);
#endif

// Mogo Cover
#if Arm_Installed_Cover
#define ArmCoverButton pros::E_CONTROLLER_DIGITAL_A
#define Arm_Cover_Down E_PISTON_RETRACTED
#define Arm_Cover_Up E_PISTON_EXTENDED
static PistonConfig arm_mogocover_piston_cfg('b', 1, E_PISTON_RETRACTED, E_PISTON_DOUBLE_ACTION);
#endif

// HighPost
#if Arm_Installed_HighPost
#define Arm_HighPost_Down E_PISTON_RETRACTED
#define Arm_HighPost_Up E_PISTON_EXTENDED
static PistonConfig arm_highpost_piston_cfg('c', 1, E_PISTON_EXTENDED, E_PISTON_DOUBLE_ACTION);
#endif

// Optical
#if Arm_Installed_Optical
static OpticalConfig arm_optical_cfg(2,75);
#endif

// Rotaiton Sensor
#if Arm_Installed_Rotation
static RotationConfig arm_rotation_cfg(7, true);
#endif











//----------------------------------------------
// Rear Clamp
//----------------------------------------------
// Compiler info
#define Rear_Installed_Optical true


// Macros
#define Rear_Clamp_Down E_PISTON_EXTENDED
#define Rear_Clamp_Up E_PISTON_RETRACTED

#define Rear_Tilter_Down E_PISTON_RETRACTED
#define Rear_Tilter_Up E_PISTON_EXTENDED
// Power
static PistonConfig rear_clamp_piston_cfg('d',1, E_PISTON_RETRACTED, E_PISTON_DOUBLE_ACTION);
static PistonConfig rear_tilter_piston_cfg('e',2, E_PISTON_RETRACTED, E_PISTON_SINGLE_ACTION);
// Sensor
static OpticalConfig rear_optical_cfg(9, 75);


//----------------------------------------------
// Ring Mech
//----------------------------------------------
static MotorConfig ring_motor_cfg(10,true,motor_gearset_600,motor_brake_coast);


#endif