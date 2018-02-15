/*
  VariableInit.cpp

   Created on: Feb 13, 2018
       Author: 5561
 */

#include "VariableInit.hpp"


/******************************************************************************
 * Function:     VariableInit
 *
 * Description:  Initialize the necessary variables when the robot switches
 *               modes (i.e. from auton to teleop).
 ******************************************************************************/
void VariableInit(Preferences *L_DriverPreferences)
  {
  T_RobotSide L_RobotSide;

  input1 = 0;
  V_WinchSpeed = 0.0;

  input1 = L_DriverPreferences->GetDouble("SetSpeed", 0.0);
  V_ProportionalGain[E_RobotSideLeft] = L_DriverPreferences->GetDouble("P_R", C_WheelSpeedPID_Gain[E_RobotSideLeft][E_PID_Proportional]);
  V_IntegralGain[E_RobotSideLeft] = L_DriverPreferences->GetDouble("I_R", C_WheelSpeedPID_Gain[E_RobotSideLeft][E_PID_Integral]);
  V_DerivativeGain[E_RobotSideLeft] = L_DriverPreferences->GetDouble("D_R", C_WheelSpeedPID_Gain[E_RobotSideLeft][E_PID_Derivative]);
  V_ProportionalGain[E_RobotSideRight] = L_DriverPreferences->GetDouble("P_L", C_WheelSpeedPID_Gain[E_RobotSideRight][E_PID_Proportional]);
  V_IntegralGain[E_RobotSideRight] = L_DriverPreferences->GetDouble("I_L", C_WheelSpeedPID_Gain[E_RobotSideRight][E_PID_Integral]);
  V_DerivativeGain[E_RobotSideRight] = L_DriverPreferences->GetDouble("D_L", C_WheelSpeedPID_Gain[E_RobotSideRight][E_PID_Derivative]);

  for (L_RobotSide = E_RobotSideLeft;
       L_RobotSide < E_RobotSideSz;
       L_RobotSide = T_RobotSide(int(L_RobotSide) + 1))
    {
    V_WheelSpeedErrorPrev[L_RobotSide] = 0.0;
    V_WheelSpeedErrorIntegral[L_RobotSide] = 0.0;
    V_WheelRPM_Raw[L_RobotSide] = 0.0;
    V_WheelRPM_Filt[L_RobotSide] = 0.0;
    V_WheelRPM_FiltPrev[L_RobotSide] = 0.0;
    V_WheelRPM_Desired[L_RobotSide] = 0.0;
    V_WheelMotorCmndPct[L_RobotSide] = 0.0;
    }
  }
