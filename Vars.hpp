/*
  Vars.hpp

   Created on: Feb 14, 2018
       Author: 5561
 */
#ifndef SRC_ROBORIO2018_VARS_HPP_
#define SRC_ROBORIO2018_VARS_HPP_

#include "enums.hpp"

extern double      V_EndGameWinchTime;
extern bool        V_LED_RainbowLatch;
extern int         V_AutonState;

extern double      V_WheelSpeedErrorPrev[E_RobotSideSz];
extern double      V_WheelSpeedErrorIntegral[E_RobotSideSz];
extern double      V_WheelRPM_Raw[E_RobotSideSz];
extern double      V_WheelRPM_Filt[E_RobotSideSz];
extern double      V_WheelRPM_FiltPrev[E_RobotSideSz];
extern double      V_WheelRPM_Desired[E_RobotSideSz];
extern double      V_WheelMotorCmndPct[E_RobotSideSz];
extern double      V_LiftMotorCmndPct;
extern double      V_IntakeArmAngleMotorCmndPct;
extern double      V_IntakeRollerMotorCmndPct;
extern double      V_HookMotorCmndPct;
extern double      V_WinchMotorCmndPct;
extern double V_RobotMotorCmndPct[E_RobotMotorSz];
extern double      V_ProportionalGain[E_RobotSideSz];
extern double      V_IntegralGain[E_RobotSideSz];
extern double      V_DerivativeGain[E_RobotSideSz];
extern double      V_Actuators[C_ActuatorsSz];
extern double      V_WheelSpeedLagFiltGain[E_RobotSideSz];
extern double      V_HookPosition;
extern double      V_HookRevolutions;
extern double      V_IntakePosition;
extern double      V_IntakeRevolutions;
extern double      LY_Axis;
extern double      RX_Axis;
extern double      GyroAngle;
extern double      Rt;
extern double      Lt;
extern double      input1;
extern double      V_WinchSpeed;
extern double      V_ArmAngleDeg;
extern E_DriveMode DriveMode;
extern double      V_Revolutions[E_RobotSideSz];
extern double      V_DistanceTraveled[E_RobotSideSz];
extern double      V_IntakeArmPulseToRev[E_ArmCmndSz];


#endif /* SRC_ROBORIO2018_VARS_HPP_ */
