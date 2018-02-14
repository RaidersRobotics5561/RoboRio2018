/*
  Output_SmartDashboard.cpp

   Created on: Feb 11, 2018
       Author: 5561
 */

#include "const.h"

/******************************************************************************
 * Function:     UpdateSmartDashboad
 *
 * Description:  Report data back to the smart dashboard.
 ******************************************************************************/
void UpdateSmartDashboad(double    L_GyroAngle,
                         LED_Mode  L_LED_Mode,
                         RoboState L_RobotState)
  {
//    double L_MatchTime = DriverStation::GetInstance().GetMatchTime();
////    int    L_Location = DriverStation::GetInstance().GetLocation();
//    bool   L_MatchEndGameFlag;
//
//    if (L_MatchTime <= K_EndMatchWarningTime &&
//        L_RobotState == C_Teleop)
//      {
//        L_MatchEndGameFlag = true;
//      }
//    else
//      {
//        L_MatchEndGameFlag = false;
//      }
//
//    /* Update the sensors on the robot: */
//    SmartDashboard::PutNumber("XValue", V_Accel.GetX());
//    SmartDashboard::PutNumber("YValue", V_Accel.GetY());
//    SmartDashboard::PutNumber("ZValue", V_Accel.GetZ());
//    SmartDashboard::PutNumber("GyroAngle", L_GyroAngle);
//
//    /* Update the state variables: */
//    SmartDashboard::PutBoolean("30 Second Warning", L_MatchEndGameFlag);
//    SmartDashboard::PutNumber("Match Time",L_MatchTime);
//    SmartDashboard::PutNumber("LED Mode", double(L_LED_Mode));
//
////    SmartDashboard::PutBoolean( "IMU_Connected",        ahrs->IsConnected());
//#ifdef GYRO2
//    SmartDashboard::PutNumber(  "IMU_Yaw",              ahrs->GetYaw());
//#endif
//
//    SmartDashboard::PutNumber( "Distance1",       V_Distance1);
//    SmartDashboard::PutNumber( "Distance2",       V_Distance2);
//    SmartDashboard::PutNumber( "Center",          V_Center);
//    SmartDashboard::PutNumber( "Blobs",           V_BlobsDetected);
//
//    SmartDashboard::PutNumber( "XB LY", V_XboxDrive.GetRawAxis(C_XB_JoystickLY));
//    SmartDashboard::PutNumber( "XB LX", V_XboxDrive.GetRawAxis(C_XB_JoystickLX));
//    SmartDashboard::PutNumber( "XB RY", V_XboxDrive.GetRawAxis(C_XB_JoystickRY));
//    SmartDashboard::PutNumber( "XB RX", V_XboxDrive.GetRawAxis(C_XB_JoystickRX));
//
//    SmartDashboard::PutNumber( "XB Gain", V_XboxDrive.GetRawAxis(C_XB_RTrigger));
//
//    SmartDashboard::PutNumber( "Auton Mode2",      (double)V_AutonPosition);
//    SmartDashboard::PutNumber( "Auton Mode3",      (double)V_AutonMode3);
//
//
//
//
//
//    SmartDashboard::PutNumber("Velocity 0",
//        _talon0->GetSelectedSensorVelocity(kPIDLoopIdx) / 12.75);
//    SmartDashboard::PutNumber("Velocity 1",
//        _talon3->GetSelectedSensorVelocity(kPIDLoopIdx) / 12.75);
//    SmartDashboard::PutNumber("Position 0",
//        _talon3->GetSelectedSensorPosition(kPIDLoopIdx) / 12.75);
//    SmartDashboard::PutNumber("GyroAngle", GyroAngle);
//    SmartDashboard::PutNumber("LY_Axis", LY_Axis);
//    SmartDashboard::PutNumber("SpeedFilt", SpeedFilt[0]);
//    SmartDashboard::PutNumber("SpeedRaw", SpeedRaw[0]);
//    SmartDashboard::PutNumber("desiredSpeed", desiredSpeed[0]);
//    SmartDashboard::PutNumber("Error", desiredSpeed[0] - SpeedFilt[0]);
//    SmartDashboard::PutNumber("Output%", output[0]);
//    SmartDashboard::PutNumber("Output%1", output[1]);

  };
