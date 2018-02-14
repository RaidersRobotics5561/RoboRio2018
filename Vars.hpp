/*
  Vars.hpp

   Created on: Feb 14, 2018
       Author: 5561
 */
#ifndef SRC_ROBORIO2018_VARS_HPP_
#define SRC_ROBORIO2018_VARS_HPP_


extern double   V_EndGameWinchTime;
extern bool     V_LED_RainbowLatch;
extern int      V_AutonState;

extern double V_WheelSpeedErrorPrev[E_RobotSideSz];
extern double V_WheelSpeedErrorIntegral[E_RobotSideSz];
extern double V_WheelRPM_Raw[E_RobotSideSz];
extern double V_WheelRPM_Filt[E_RobotSideSz];
extern double V_WheelRPM_FiltPrev[E_RobotSideSz];
extern double V_WheelRPM_Desired[E_RobotSideSz];
extern double V_WheelMotorCmndPct[E_RobotSideSz];
extern double V_ProportionalGain[E_RobotSideSz];
extern double V_IntegralGain[E_RobotSideSz];
extern double V_DerivativeGain[E_RobotSideSz];
extern int kSlotIdx = 0;
extern int kPIDLoopIdx = 0;
extern int kTimeoutMs = 10;
extern double TargetSpeed = 0;
extern double LY_Axis;
extern double RX_Axis;
extern double GyroAngle;
extern double Rt;
extern double Lt;
extern double IntergalR = 0;
extern double input1 = 0;
extern double DesiredSpeedPrev = 0;
extern double V_WinchSpeed = 0.0;


#endif /* SRC_ROBORIO2018_VARS_HPP_ */
