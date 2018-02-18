/*
  Read_Encoder.cpp

   Created on: Feb 18, 2018
       Author: 5561
 */

#include "Const.h"
#include "Vars.hpp"
#include "Enums.hpp"

/******************************************************************************
 * Function:     Read_Sensors
 *
 * Description:  Run all of the sensor decoding logic.
 ******************************************************************************/
void Read_Sensors(TalonSRX  *L_DriveMortorCtrlLeft,
                  TalonSRX  *L_DriveMortorCtrlRight,
                  Counter   *L_ArmEncoder,
                  double    *L_ArmAngleDeg,
                  T_ArmCmnd  L_ArmCmndPrev)
  {
//  T_RobotSide L_RobotSide;
//  int         L_ArmEncoderCount;
//
//  /* Ok, let's first read the wheel speeds: */
//  V_WheelRPM_Raw[E_RobotSideLeft]  = L_DriveMortorCtrlLeft->GetSelectedSensorVelocity(K_PIDLoopIdx) / 12.75;
//
//  V_WheelRPM_Raw[E_RobotSideRight] = (L_DriveMortorCtrlRight->GetSelectedSensorVelocity(K_PIDLoopIdx) / 12.75) * -1;
//
//  /* Ok, let's filter the speeds */
//  for (L_RobotSide = E_RobotSideLeft; L_RobotSide < E_RobotSideSz;
//       L_RobotSide = T_RobotSide(int(L_RobotSide) + 1))
//    {
//    V_WheelRPM_Filt[L_RobotSide] = LagFilter(V_WheelSpeedLagFiltGain[L_RobotSide],
//                                             V_WheelRPM_Raw[L_RobotSide],
//                                             V_WheelRPM_FiltPrev[L_RobotSide]);
//
//    V_WheelRPM_FiltPrev[L_RobotSide] = V_WheelRPM_Filt[L_RobotSide];
//
//    V_Revolutions[L_RobotSide] = V_WheelRPM_Raw[L_RobotSide] / C_WheelPulsetoRev[L_RobotSide];
//    }
//
//  /* Great, now lets read the counts from the arm encoder.  Remember, the encoder will change the rate at which it
//   * increases counts base on direction.... Also, this is a non directional counter...*/
//  L_ArmEncoderCount = L_ArmEncoder->Get();

  }





