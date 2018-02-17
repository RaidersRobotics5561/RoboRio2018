/*
  Input_Controller.cpp

   Created on: Feb 16, 2018
       Author: 5561
 */

#include "const.h"
#include "Calibrations.hpp"

/******************************************************************************
 * Function:     DtrmnControllerMapping
 *
 * Description:  Map the controller to the desired inputs.
 ******************************************************************************/
void DtrmnControllerMapping(Joystick *L_Joystick1,
                            Joystick *L_Joystick2)
  {
  double L_DrivePctLeft;
  double L_DrivePctRight;
  double L_IntakeArmAngle;
  double L_IntakeRollers;
  double L_IntakeLift;
  double L_Winch;
  double L_Hook;

  L_DrivePctLeft = L_Joystick1->GetRawAxis(1); // Left stick
  L_DrivePctRight = L_Joystick1->GetRawAxis(5); // Right stick
  L_Hook = L_Joystick1->GetRawAxis(3) - L_Joystick1->GetRawAxis(2); // Combination of right and left trigger

  if (L_Joystick2->GetRawButton(1) == true)
    {
    L_IntakeRollers = K_IntakeRollers;
    }
  else if (L_Joystick2->GetRawButton(2) == true)
    {
    L_IntakeRollers = -K_IntakeRollers;
    }

  L_IntakeArmAngle = L_Joystick2->GetRawAxis(1);
  L_IntakeLift = L_Joystick2->GetRawAxis(3) - L_Joystick2->GetRawAxis(2);
  L_Winch = L_Joystick2->GetRawAxis(5);

//  C_DriveMotorL,
//  C_DriveMotorR,
//  C_IntakeArmAngle,
//  C_IntakeRollers,
//  C_IntakeLift,
//  C_Winch,
//  C_Hook,
//  V_Actuators[C_ActuatorsSz];
  }

