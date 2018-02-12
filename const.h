/*
 * const.h
 *
 *  Created on: Feb 1, 2018
 *      Author: wesat
 */

#ifndef SRC_CONST_H_
#define SRC_CONST_H_

//typedef enum {
//	E_StateOff, E_StateForward, E_StateRotate, E_StateEnd
//} T_State;

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
		{ 0.06, 0.01, 0.00 }, //LEFT
		{ 0.06, 0.01, 0.00 }}; //RIGHT

const double C_WheelspeedIntergalLimit[E_RobotSideSz][E_IntergalLimitSz] = {
		// UPPER LOWER
		{ 0.1, -0.1}, //LEFT
		{ 0.1, -0.1}  //RIGHT
};

const double C_WheelSpeedLagFilterGain[E_RobotSideSz] =
		{ 0.01, 0.01};

const double C_WheelPulsetoRev[E_RobotSideSz] =
		{  370,  370};

const double C_WheelDiameter[E_RobotSideSz] =
		{ 7, 7};

const double C_speedGain = 10;
const double C_SpeedFilterGain = 0.01;
const double C_PI = 3.14159265358979;

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

#endif /* SRC_CONST_H_ */
