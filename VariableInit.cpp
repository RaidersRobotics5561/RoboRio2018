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
void VariableInit(Preferences *L_DriverPreferences,
                  Counter     *mCounter)
  {
  T_RobotSide L_RobotSide;
  T_RobotMotor L_RobotMotor;

  input1 = 0;
  V_WinchSpeed = 0.0;
  RX_Axis = 0.0;
  LY_Axis = 0.0;

  mCounter->Reset();
  input1 = L_DriverPreferences->GetDouble("SetSpeed", 0.0);
  V_ProportionalGain[E_RobotSideLeft] = L_DriverPreferences->GetDouble("P_L", C_WheelSpeedPID_Gain[E_RobotSideLeft][E_PID_Proportional]);
  V_IntegralGain[E_RobotSideLeft] = L_DriverPreferences->GetDouble("I_L", C_WheelSpeedPID_Gain[E_RobotSideLeft][E_PID_Integral]);
  V_DerivativeGain[E_RobotSideLeft] = L_DriverPreferences->GetDouble("D_L", C_WheelSpeedPID_Gain[E_RobotSideLeft][E_PID_Derivative]);
  V_ProportionalGain[E_RobotSideRight] = L_DriverPreferences->GetDouble("P_R", C_WheelSpeedPID_Gain[E_RobotSideRight][E_PID_Proportional]);
  V_IntegralGain[E_RobotSideRight] = L_DriverPreferences->GetDouble("I_R", C_WheelSpeedPID_Gain[E_RobotSideRight][E_PID_Integral]);
  V_DerivativeGain[E_RobotSideRight] = L_DriverPreferences->GetDouble("D_R", C_WheelSpeedPID_Gain[E_RobotSideRight][E_PID_Derivative]);
  V_WheelSpeedLagFiltGain[E_RobotSideRight] = L_DriverPreferences->GetDouble("Lag_R", C_WheelSpeedLagFilterGain[E_RobotSideRight]);
  V_WheelSpeedLagFiltGain[E_RobotSideLeft] = L_DriverPreferences->GetDouble("Lag_L", C_WheelSpeedLagFilterGain[E_RobotSideLeft]);

  V_IntakeArmPulseToRev[E_ArmCmndOff] =  L_DriverPreferences->GetDouble("ArmPulseOff", K_IntakeArmPulseToRev[E_ArmCmndOff]);
  V_IntakeArmPulseToRev[E_ArmCmndUp] =  L_DriverPreferences->GetDouble("ArmPulseUp", K_IntakeArmPulseToRev[E_ArmCmndUp]);
  V_IntakeArmPulseToRev[E_ArmCmndDwn] =  L_DriverPreferences->GetDouble("ArmPulseDwn", K_IntakeArmPulseToRev[E_ArmCmndDwn]);

  V_ArmAngleDeg = 0.0;
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
    }

  for (L_RobotMotor = E_RobotMotorLeftWheel;
      L_RobotMotor < E_RobotMotorSz;
      L_RobotMotor = T_RobotMotor(int(L_RobotMotor) + 1))
    {
    V_RobotMotorCmndPct[L_RobotMotor] = 0.0;
    }


  }
