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
#include "Enums.hpp"


const double C_ExeTime = 0.01; // Execution rate of the Roborio controller

const double C_WheelPulsetoRev[E_RobotSideSz] =
    {  370,  370};

const double C_WheelDiameter[E_RobotSideSz] =
    { 7, 7};

const double C_PI = 3.14159265358979;

#endif /* SRC_CONST_H_ */
