/*
  Read_Encoder.cpp

   Created on: Feb 18, 2018
       Author: 5561
 */

#include "Const.h"
#include "SignalFilter.hpp"
#include "Vars.hpp"
#include "Enums.hpp"
#include "Calibrations.hpp"

/******************************************************************************
 * Function:     Read_Sensors
 *
 * Description:  Run all of the sensor decoding logic.
 ******************************************************************************/
void Read_Sensors(TalonSRX  *L_DriveMortorCtrlLeft,
                  TalonSRX  *L_DriveMortorCtrlRight,
                  TalonSRX  *L_Intake,
                  TalonSRX  *L_Hook,
                  Counter   *L_ArmEncoder,
                  ADXRS450_Gyro *Gyro,
                  double    *L_ArmAngleDeg,
                  T_ArmCmnd  L_ArmCmndPrev,
                  T_ArmCmnd  L_ArmCmndPrevPrev)
  {
  T_RobotSide L_RobotSide;
  double      L_ArmEncoderCount = 0.0;
  double      L_ArmAngle;

  /* Ok, let's first read the wheel speeds: */
  V_WheelRPM_Raw[E_RobotSideLeft]  = L_DriveMortorCtrlLeft->GetSelectedSensorVelocity(K_PIDLoopIdx) / K_WheelPulseToRev;

  V_WheelRPM_Raw[E_RobotSideRight] = (L_DriveMortorCtrlRight->GetSelectedSensorVelocity(K_PIDLoopIdx) / K_WheelPulseToRev) * -1;

  /* Ok, let's filter the speeds */
  for (L_RobotSide = E_RobotSideLeft; L_RobotSide < E_RobotSideSz;
       L_RobotSide = T_RobotSide(int(L_RobotSide) + 1))
    {
    V_WheelRPM_Filt[L_RobotSide] = LagFilter(V_WheelSpeedLagFiltGain[L_RobotSide],
                                             V_WheelRPM_Raw[L_RobotSide],
                                             V_WheelRPM_FiltPrev[L_RobotSide]);

    V_WheelRPM_FiltPrev[L_RobotSide] = V_WheelRPM_Filt[L_RobotSide];

    V_Revolutions[L_RobotSide] = V_WheelRPM_Raw[L_RobotSide] / C_WheelPulsetoRev[L_RobotSide];

    V_DistanceTraveled[L_RobotSide] = V_Revolutions[L_RobotSide] * C_PI * C_WheelDiameter[L_RobotSide];
    }

  /* Great, now lets read the counts from the arm encoder.  Remember, the encoder will change the rate at which it
   * increases counts base on direction.... Also, this is a non directional counter...*/
  if ((L_ArmCmndPrev != L_ArmCmndPrevPrev) ||
      (L_ArmCmndPrev == E_ArmCmndOff))
    {
    /* There appears to have been a direction change!  Reset the counter. */
    L_ArmEncoder->Reset();
    }
  else if (L_ArmCmndPrev < E_ArmCmndSz)
    {
    /* Just check to make sure we are within bounds (i.e. not size) */
    L_ArmEncoderCount = (double)L_ArmEncoder->Get();
    L_ArmAngle        = L_ArmEncoderCount * V_IntakeArmPulseToRev[L_ArmCmndPrev];
    if (L_ArmCmndPrev == E_ArmCmndUp)
      {
      *L_ArmAngleDeg += L_ArmAngle;
      }
    else
      {
      *L_ArmAngleDeg -= L_ArmAngle;
      }
    L_ArmEncoder->Reset();
    }

  /* Now let's figure out the position of the hook (arrgh!!!) */
  /* Convert the pulses to hook distance traveled: */
  V_HookPosition = fabs(L_Hook->GetSelectedSensorPosition(K_PIDLoopIdx) * K_HookPulseToTravel);


  /* Great, let's finally figure out the intake vertical position: */
  /* Next, convert the revolutions to hook distance traveled: */
  V_IntakePosition = fabs(L_Intake->GetSelectedSensorPosition(K_PIDLoopIdx) * K_IntakePulseToTravel);
  }
