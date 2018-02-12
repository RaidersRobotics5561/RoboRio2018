/*
 * const.h
 *
 *  Created on: Feb 1, 2018
 *      Author: wesat
 */

#ifndef SRC_CONST_H_
#define SRC_CONST_H_

#include "WPILib.h"
#include "Joystick.h"
#include "ctre/Phoenix.h"
#include "ADXRS450_Gyro.h"
#include "DigitalOutput.h"

extern double   V_EndGameWinchTime;
extern bool     V_LED_RainbowLatch;
extern int      V_AutonState;


 typedef enum
 {
   C_BlinkR,
   C_BlinkL,
   C_BlinkSz
 }Blinker;

 typedef enum
 {
   LED_Color_Red,
   LED_Color_Blue,
   LED_Color_Green,
   LED_Color_White,
   LED_Color_Purple,
   LED_Color_Yellow,
   LED_Color_Pink,
   LED_Color_Orange,
   LED_Color_Rainbow, // This is meant to indicate when a random mixture of colors are desired
   LED_Color_Multi,   // This is meant to indicate when it is desired to have the colors cycle through all of the avaiable colors above rainbow
   LED_Color_Black    // This is more of an "off mode", must remain at end of enum
 } LED_Color;

 typedef enum
 {
   LED_Mode0,
   LED_Mode1,
   LED_Mode2,
   LED_Mode3,
   LED_Mode4,
   LED_Mode5,
   LED_Mode6,
   LED_Mode7,
   LED_Mode8,
   LED_Mode9,
   LED_Mode10,
   LED_Mode11,
   LED_Mode12,
   LED_Mode13,
   LED_Mode14,
   LED_Mode15,
   LED_ModeSz
 } LED_Mode;

 typedef enum
 {
   C_Disabled,
   C_Auton,
   C_Teleop,
   C_Test,
   C_Null
 }RoboState;

typedef enum {
	E_RobotSideLeft, E_RobotSideRight, E_RobotSideSz
} T_RobotSide;

typedef enum {
	E_PID_Proportional, E_PID_Integral, E_PID_Derivative, E_PID_Sz
} T_PID;

typedef enum {
	E_IntergalUpperLimit, E_IntergalLowerLimit, E_IntergalLimitSz
} T_IntergalLimit;

const double C_WheelSpeedPID_Gain[E_RobotSideSz][E_PID_Sz] = {
		// P    I    D
		{ 0.01, 0.001, 0.0 }, //LEFT
		{ 0.01, 0.001, 0.0 }}; //RIGHT

const double C_WheelspeedProportionalLimit[E_RobotSideSz][E_IntergalLimitSz] = {
    // UPPER LOWER
    { 1.0, -1.0}, //LEFT
    { 1.0, -1.0}  //RIGHT
};

const double C_WheelspeedIntergalLimit[E_RobotSideSz][E_IntergalLimitSz] = {
    // UPPER LOWER
    { 0.6, -0.6}, //LEFT
    { 0.6, -0.6}  //RIGHT
};

const double C_WheelspeedDerivativeLimit[E_RobotSideSz][E_IntergalLimitSz] = {
    // UPPER LOWER
    { 1.0, -1.0}, //LEFT
    { 1.0, -1.0}  //RIGHT
};

const double C_WheelspeedCmndLimit[E_RobotSideSz][E_IntergalLimitSz] = {
    // UPPER LOWER
    { 1.0, -1.0}, //LEFT
    { 1.0, -1.0}  //RIGHT
};



const double C_WheelSpeedLagFilterGain[E_RobotSideSz] =
		{ 0.6, 0.6};

const double K_EndMatchWarningTime  =  30;    // This is the expected time remaining that the robot will warn the
const double K_WinchOnThreshold     =   0.1; // Threshold above which the winch is considered to be on.
const double K_LED_WinchOnTime      =   3.0;  // This is the amount of accumulated time that the winch needs to be commanded on at the end of the game in order to trigger the rainbow effect

const double K_DesiredDriveSpeedAxis[20] = {-1.00,
                                            -0.90,
                                            -0.80,
                                            -0.70,
                                            -0.60,
                                            -0.50,
                                            -0.40,
                                            -0.30,
                                            -0.20,
                                            -0.10,
                                             0.00,
                                             0.10,
                                             0.20,
                                             0.30,
                                             0.40,
                                             0.50,
                                             0.60,
                                             0.70,
                                             0.80,
                                             0.90};

const double K_DesiredDriveSpeed[20] = {-100.0,  //-1.00
                                        -50.0,  //-0.90
                                        -25.0,  //-0.80
                                        -21.0,  //-0.70
                                        -17.0,  //-0.60
                                        -13.0,  //-0.50
                                         -9.0,  //-0.40
                                         -5.0,  //-0.30
                                         -2.0,  //-0.20
                                          0.0,  //-0.10
                                          0.0,  // 0.00
                                          0.0,  // 0.10
                                          2.0,  // 0.20
                                          5.0,  // 0.30
                                          9.0,  // 0.40
                                         13.0,  // 0.50
                                         17.0,  // 0.60
                                         21.0,  // 0.70
                                         50.0,  // 0.80
                                         100.0}; // 0.90

const double C_speedGain = 10;
const double C_SpeedFilterGain = 0.01;

//const double C_ErrorP_L = 0.06;
//const double C_ErrorI_L = 0.01;
//const double C_ErrorD_L = 0.01;
//const double C_IntergalUpperLimit_L = 0.1;
//const double C_IntergalLowerLimit_L = -0.1;
//const double C_FiltGain_L = 0.01;

//const double C_ErrorP_R = 0.06;
//const double C_ErrorI_R = 0.01;
//const double C_ErrorD_R = 0.01;
//const double C_IntergalUpperLimit_R = 0.1;
//const double C_IntergalLowerLimit_R = -0.1;
//const double C_FiltGain_R = 0.01;

const double C_ExeTime = 0.01;
const double C_ControllerUpdateRate = 0.01;  // Execution rate of the Roborio controller


extern double Control_PID(double  L_DesiredSpeed,
                          double  L_CurrentSpeed,
                          double *L_ErrorPrev,
                          double *L_IntegralPrev,
                          double  L_ProportionalGx,
                          double  L_IntegralGx,
                          double  L_DerivativeGx,
                          double  L_ProportionalUpperLimit,
                          double  L_ProportionalLowerLimit,
                          double  L_IntegralUpperLimit,
                          double  L_IntegralLowerLimit,
                          double  L_DerivativeUpperLimit,
                          double  L_DerivativeLowerLimit,
                          double  L_OutputUpperLimit,
                          double  L_OutputLowerLimit);

extern double LookUp1D_Table(const double *L_X_Axis,
                             const double *L_TableData1D,
                                   int     L_AxisSize,
                                   int     L_CalArraySize,
                                   double  L_Input);

#endif /* SRC_CONST_H_ */
