/*
  Output_SmartDashboard.cpp

   Created on: Feb 11, 2018
       Author: 5561
 */

#include "Output_SmartDashboard.hpp"


/******************************************************************************
 * Function:     UpdateSmartDashboad
 *
 * Description:  Report data back to the smart dashboard.
 ******************************************************************************/
void UpdateSmartDashboad(void)
  {
    SmartDashboard::PutNumber("SpeedRawLeft", V_WheelRPM_Raw[E_RobotSideLeft]);
    SmartDashboard::PutNumber("SpeedRawRight", V_WheelRPM_Raw[E_RobotSideRight]);
    SmartDashboard::PutNumber("RightSide", V_RobotMotorCmndPct[E_RobotMotorRightWheel]);
    SmartDashboard::PutNumber("LeftSide", V_RobotMotorCmndPct[E_RobotMotorLeftWheel]);
    SmartDashboard::PutNumber("LeftDesired", V_WheelRPM_Desired[E_RobotSideLeft]);
    SmartDashboard::PutNumber("RightDesired", V_WheelRPM_Desired[E_RobotSideRight]);
    SmartDashboard::PutNumber("LeftFilt", V_WheelRPM_Filt[E_RobotSideLeft]);
    SmartDashboard::PutNumber("RightFilt", V_WheelRPM_Filt[E_RobotSideRight]);
    SmartDashboard::PutNumber("LeftSEV", V_WheelSpeedErrorPrev[E_RobotSideLeft]);
    SmartDashboard::PutNumber("RightSEV", V_WheelSpeedErrorPrev[E_RobotSideRight]);
    SmartDashboard::PutNumber("LeftSEI", V_WheelSpeedErrorIntegral[E_RobotSideLeft]);
    SmartDashboard::PutNumber("RightSEI", V_WheelSpeedErrorIntegral[E_RobotSideRight]);
    SmartDashboard::PutNumber("DriveMode",(double)DriveMode);

    SmartDashboard::PutNumber("Arm Angle",     V_ArmAngleDeg);

    SmartDashboard::PutNumber("Hook Position",     V_HookPosition);
    SmartDashboard::PutNumber("Hook Desired",     V_HookLiftHeightDesired);
    SmartDashboard::PutNumber("Hook Motor", V_RobotMotorCmndPct[E_RobotMotorHook]);


    SmartDashboard::PutNumber("Intake Cmnd",   V_RobotUserCmndPct[E_RobotUserCmndLift]);
    SmartDashboard::PutNumber("Intake Position",   V_IntakePosition);
    SmartDashboard::PutNumber("Intake Desired Position",   V_IntakeLiftHeightDesired);
    SmartDashboard::PutNumber("Intake Lift Motor", V_RobotMotorCmndPct[E_RobotMotorLift]);
  };
