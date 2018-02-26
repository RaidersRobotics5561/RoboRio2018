/*
  Control_Auton.cpp

   Created on: Feb 23, 2018
       Author: 5561
 */

#include "Enums.hpp"
#include "const.h"
#include "Control_Auton.hpp"
#include "Calibrations.hpp"
#include "Vars.hpp"
#include "Control_PID.hpp"

double V_RollerTimer;
double V_IntakeArmTimer;

/******************************************************************************
 * Function:     DtrmnAutonOption
 *
 * Description:  Determine what path the robot will travel in during the auton
 *               mode.  This should only need to be called once when starting
 *               auton mode.:
 *
 *                   E_AutonOpt0  - LLL
 *                   E_AutonOpt1  - LLR
 *                   E_AutonOpt2  - RLL
 *                   E_AutonOpt3  - RLR
 *                   E_AutonOpt4  - MLL
 *                   E_AutonOpt5  - MLR
 *                   E_AutonOpt6  - LLF
 *                   E_AutonOpt7  - LLS
 *                   E_AutonOpt8  - LRS
 *                   E_AutonOpt9  - LRF
 *                   E_AutonOpt10 - MLF
 *                   E_AutonOpt11 - MLS
 *                   E_AutonOpt12 - MRS
 *                   E_AutonOpt13 - MRF
 *                   E_AutonOpt14 - RLF
 *                   E_AutonOpt15 - RLS
 *                   E_AutonOpt16 - RRS
 *                   E_AutonOpt17 - RRF
 *
 ******************************************************************************/
T_AutonOpt DtrmnAutonOption(T_RobotSide     L_AutonTargetSwitch,
                            T_RobotSide     L_AutonTargetScale,
                            T_AutonStartPos L_AutonStartPos,
                            T_AutonEndPos   L_AutonEndPos)
  {
  T_AutonOpt L_AutonOption;

  if(L_AutonEndPos == E_AutonEndPosSwFront)
    {
    if (L_AutonStartPos == E_AutonStartPosLeft)
      {
      if (L_AutonTargetSwitch == E_RobotSideLeft)
        {
//        L_AutonOption = E_AutonOpt0;
        }
      else
        {
//        L_AutonOption = E_AutonOpt0;
        }
      }
    else if (L_AutonStartPos == E_AutonStartPosRight)
      {
      if (L_AutonTargetSwitch == E_RobotSideLeft)
        {
//        L_AutonOption = E_AutonOpt0;
        }
      else
        {
//        L_AutonOption = E_AutonOpt0;
        }
      }
    else // L_AutonStartPos == E_AutonStartPosRight
      {
      if (L_AutonTargetSwitch == E_RobotSideLeft)
        {
//        L_AutonOption = E_AutonOpt0;
        }
      else
        {
//        L_AutonOption = E_AutonOpt0;
        }
      }
    }
  else if(L_AutonEndPos == E_AutonEndPosSwSide)
    {
    if (L_AutonStartPos == E_AutonStartPosLeft)
      {
      if (L_AutonTargetSwitch == E_RobotSideLeft)
        {
//        L_AutonOption = E_AutonOpt0;
        }
      else
        {
//        L_AutonOption = E_AutonOpt0;
        }
      }
    else if (L_AutonStartPos == E_AutonStartPosRight)
      {
      if (L_AutonTargetSwitch == E_RobotSideLeft)
        {
//        L_AutonOption = E_AutonOpt0;
        }
      else
        {
//        L_AutonOption = E_AutonOpt0;
        }
      }
    else // L_AutonStartPos == E_AutonStartPosRight
      {
      if (L_AutonTargetSwitch == E_RobotSideLeft)
        {
//        L_AutonOption = E_AutonOpt0;
        }
      else
        {
//        L_AutonOption = E_AutonOpt0;
        }
      }
    }
  else /*  E_AutonEndPosScale  */
    {
    if (L_AutonStartPos == E_AutonStartPosLeft)
      {
      if (L_AutonTargetScale == E_RobotSideLeft)
        {
//        L_AutonOption = E_AutonOpt0;
        }
      else
        {
//        L_AutonOption = E_AutonOpt0;
        }
      }
    else if (L_AutonStartPos == E_AutonStartPosRight)
      {
      if (L_AutonTargetScale == E_RobotSideLeft)
        {
//        L_AutonOption = E_AutonOpt0;
        }
      else
        {
//        L_AutonOption = E_AutonOpt0;
        }
      }
    else // L_AutonStartPos == E_AutonStartPosRight
      {
      if (L_AutonTargetScale == E_RobotSideLeft)
        {
//        L_AutonOption = E_AutonOpt0;
        }
      else
        {
//        L_AutonOption = E_AutonOpt0;
        }
      }
    }

  L_AutonOption = E_AutonOpt0; // FOR TEST ONLY!!

  return (L_AutonOption);
  }

/******************************************************************************
 * Function:     DtrmnActuatorComplete
 *
 * Description:  Determine when an actuator has reached its desired end point.
 *
 ******************************************************************************/
bool DtrmnActuatorComplete(double     L_CntrlVal,
                           double     L_MeasuredVal,
                           double     L_Deadband,
                           double    *L_DebounceTimer,
                           double     L_DebounceThreshold)
  {
  bool   L_ControlComplete = false;
  double L_DeadbandHigh;
  double L_DeadbandLow;

  L_DeadbandHigh = L_CntrlVal + L_Deadband;
  L_DeadbandLow  = L_CntrlVal - L_Deadband;

  if ((L_MeasuredVal >= L_DeadbandLow) &&
      (L_MeasuredVal <= L_DeadbandHigh))
    {
    *L_DebounceTimer += C_ExeTime;

    if (*L_DebounceTimer >= L_DebounceThreshold)
      {
      *L_DebounceTimer = L_DebounceThreshold;
      L_ControlComplete = true;
      }
    }
  else
    {
    *L_DebounceTimer = 0.0;
    }

  return (L_ControlComplete);
  }



/******************************************************************************
 * Function:     CntrlAutonDesiredSpeed
 *
 * Description:  Determine the desired speed for a given actuator(s) while in
 *               Auton.  The intent is to ramp the requested speed up from zero,
 *               to a max value, then ramp down to a minimum value once we get
 *               near the end.
 *
 ******************************************************************************/
double CntrlAutonDesiredSpeed(double            L_K_TotalPlannedTravel,
                              double            L_MeasuredTravel,
                              double            L_DesiredSpeedPrev,
                              double            L_K_RampDownTravel,
                              double            L_K_MinSpeed,
                              double            L_K_MaxSpeed,
                              double            L_K_SpeedRamp,
                              double            L_DebounceTime,
                              bool              L_TargetMet)
  {
  double L_DesiredSpeed     = 0.0;
  double L_RampDownDistance = 0.0;

  /* Find the angle at which we want to start ramping out the desired speed. */
  L_RampDownDistance = L_K_TotalPlannedTravel - L_K_RampDownTravel;

  if (L_RampDownDistance < 0.0)
    {
    /* Limit to zero */
    L_RampDownDistance = 0.0;
    }


  if ((L_MeasuredTravel >= L_RampDownDistance) &&
      (L_DesiredSpeedPrev > L_K_MinSpeed))
    {
    /* We have reached the point at which we need to start slowing down
     * (i.e. let's put the brakes on!). */
    L_DesiredSpeed = L_DesiredSpeedPrev - (L_K_SpeedRamp * C_ExeTime);
    }
  else if (((L_MeasuredTravel < L_RampDownDistance) &&
            (L_DesiredSpeedPrev < L_K_MaxSpeed)) ||
           (L_DesiredSpeedPrev < L_K_MinSpeed))
    {
    /* We still have a way to go or we haven't reached our minimum speed.
     * Let's speed up! */
    L_DesiredSpeed = L_DesiredSpeedPrev + (L_K_SpeedRamp * C_ExeTime);
    }
  else if (L_DesiredSpeedPrev < L_K_MinSpeed)
    {
    /* A catch all.  Unless we are ramping up the speed, we should limit to
     * a minimum. */
    L_DesiredSpeed = L_K_MinSpeed;
    }
  else if (L_DesiredSpeedPrev >= L_K_MaxSpeed)
    {
    /* Another defensive programming measure.  We want to make sure we don't
     * go past the max value! */
    L_DesiredSpeed = L_K_MaxSpeed;
    }

  if (L_TargetMet == true)
    {
    /* We have met our target and the debounce timers have run.
     * Let's request 0 speed. */
    L_DesiredSpeed = 0.0;
    }
  else if (L_MeasuredTravel > L_K_TotalPlannedTravel)
    {
    /*Opps!  Looks like we overshot our target.  Lets back up.*/
    L_DesiredSpeed = -L_K_MinSpeed;
    }
  else if (L_DebounceTime > 0.0)
    {
    /* The debounce timer seems to be running.  This must mean that we are close.
     * Let's run at the min speed to make sure we don't overshoot. */
    L_DesiredSpeed = L_K_MinSpeed;
    }

  return (L_DesiredSpeed);
  }


/******************************************************************************
 * Function:     CntrlAutonDrive
 *
 * Description:  Control the drivetrain in auton.
 *
 ******************************************************************************/
bool CntrlAutonDrive(T_Actuator       L_CntrlActuator,
                     double           L_AutonTarget)
  {
  bool        L_ControlComplete = false;
  T_RobotSide L_RobotSide       = E_RobotSideLeft;

  if (L_CntrlActuator == E_ActuatorDriveEncoder)
    {
    L_ControlComplete = DtrmnActuatorComplete( L_AutonTarget,
                                               V_DistanceTraveledAvg,
                                               K_AutonDriveDistanceDeadband,
                                              &V_AutonWheelDebounceTimer[E_RobotSideLeft],
                                               K_AutonDebounceThreshold);

    V_WheelRPM_Desired[E_RobotSideLeft] = CntrlAutonDesiredSpeed(L_AutonTarget,
                                                                 V_DistanceTraveledAvg,
                                                                 V_WheelRPM_Desired[E_RobotSideLeft],
                                                                 K_AutonDriveDistanceToSlow,
                                                                 K_AutonDriveMinSpeed,
                                                                 K_AutonDriveMaxSpeed,
                                                                 K_AutonDriveSpeedRamp,
                                                                 V_AutonWheelDebounceTimer[E_RobotSideLeft],
                                                                 L_ControlComplete);

    /* Since we are driving straight, we will have a desired speed that will be the same for both sides: */
    V_WheelRPM_Desired[E_RobotSideRight] = V_WheelRPM_Desired[E_RobotSideLeft];
    }
  else if (L_CntrlActuator == E_ActuatorDriveUltraSonic)
    {
    for (L_RobotSide = E_RobotSideLeft;
         L_RobotSide < E_RobotSideSz;
         L_RobotSide = T_RobotSide(int(L_RobotSide) + 1))
      {
      L_ControlComplete = DtrmnActuatorComplete( L_AutonTarget,
                                                 V_DistanceTraveled[L_RobotSide],
                                                 K_AutonDriveDistanceUltraDeadband,
                                                &V_AutonWheelDebounceTimer[L_RobotSide],
                                                 K_AutonDebounceThreshold);

      V_WheelRPM_Desired[L_RobotSide] = CntrlAutonDesiredSpeed(L_AutonTarget,
                                                               V_DistanceTraveledAvg,
                                                               V_WheelRPM_Desired[L_RobotSide],
                                                               K_AutonDriveDistanceUltraToSlow,
                                                               K_AutonDriveMinSpeedUltra,
                                                               K_AutonDriveMaxSpeed,
                                                               K_AutonDriveSpeedRamp,
                                                               V_AutonWheelDebounceTimer[L_RobotSide],
                                                               L_ControlComplete);
      }
    }
  else if (L_CntrlActuator == E_ActuatorRotate)
    {
    L_ControlComplete = DtrmnActuatorComplete( L_AutonTarget,
                                               V_DistanceTraveledAvg,
                                               K_AutonDriveDistanceDeadband,
                                              &V_AutonWheelDebounceTimer[E_RobotSideLeft],
                                               K_AutonDebounceThreshold);

    V_WheelRPM_Desired[E_RobotSideLeft] = CntrlAutonDesiredSpeed(L_AutonTarget,
                                                                 V_GyroAngleRelative,
                                                                 V_WheelRPM_Desired[E_RobotSideLeft],
                                                                 K_AutonRotateAngleToSlow,
                                                                 K_AutonDriveMinSpeed,
                                                                 K_AutonDriveMaxSpeed,
                                                                 K_AutonDriveSpeedRamp,
                                                                 V_AutonWheelDebounceTimer[E_RobotSideLeft],
                                                                 L_ControlComplete);

    /* Since we are driving straight, we will have a desired speed that will be the same for both sides: */
    V_WheelRPM_Desired[E_RobotSideRight] = -V_WheelRPM_Desired[E_RobotSideLeft];
    }

  for (L_RobotSide = E_RobotSideLeft;
       L_RobotSide < E_RobotSideSz;
       L_RobotSide = T_RobotSide(int(L_RobotSide) + 1))
    {
    V_RobotMotorCmndPct[E_RobotMotorLeftWheel]  = Control_PID(V_WheelRPM_Desired[L_RobotSide],
                                                              V_WheelRPM_Filt[L_RobotSide],
                                                              &V_WheelSpeedErrorPrev[L_RobotSide],
                                                              &V_WheelSpeedErrorIntegral[L_RobotSide],
                                                              V_WheelProportionalGain[L_RobotSide],
                                                              V_WheelIntegralGain[L_RobotSide],
                                                              V_WheelDerivativeGain[L_RobotSide],
                                                              C_WheelspeedProportionalLimit[L_RobotSide][E_IntergalUpperLimit],
                                                              C_WheelspeedProportionalLimit[L_RobotSide][E_IntergalLowerLimit],
                                                              C_WheelspeedIntergalLimit[L_RobotSide][E_IntergalUpperLimit],
                                                              C_WheelspeedIntergalLimit[L_RobotSide][E_IntergalLowerLimit],
                                                              C_WheelspeedDerivativeLimit[L_RobotSide][E_IntergalUpperLimit],
                                                              C_WheelspeedDerivativeLimit[L_RobotSide][E_IntergalLowerLimit],
                                                              C_WheelspeedCmndLimit[L_RobotSide][E_IntergalUpperLimit],
                                                              C_WheelspeedCmndLimit[L_RobotSide][E_IntergalLowerLimit]);
    }


  return (L_ControlComplete);
  }

/******************************************************************************
 * Function:     CntrlAutonLift
 *
 * Description:  Control the lift in auton.
 *
 ******************************************************************************/
bool CntrlAutonLift(T_Actuator       L_CntrlActuator,
                    double           L_AutonTarget)
  {
  bool        L_ControlComplete = false;

  L_ControlComplete = DtrmnActuatorComplete( L_AutonTarget,
                                             V_IntakePosition,
                                             K_AutonIntakeDistanceDeadband,
                                            &V_AutonIntakeLiftDebounceTimer,
                                             K_AutonDebounceThreshold);

  V_IntakeLiftHeightDesired = CntrlAutonDesiredSpeed(L_AutonTarget,
                                                     V_IntakePosition,
                                                     V_IntakeLiftHeightDesired,
                                                     K_AutonIntakeDistanceToSlow,
                                                     K_AutonIntakeMinSpeed,
                                                     K_AutonIntakeMaxSpeed,
                                                     K_AutonIntakeSpeedRamp,
                                                     V_AutonIntakeLiftDebounceTimer,
                                                     L_ControlComplete);

  V_RobotMotorCmndPct[E_RobotMotorLift]  = Control_PID( V_IntakeLiftHeightDesired,
                                                        V_IntakePosition,
                                                       &V_IntakePositionErrorPrev,
                                                       &V_IntakePositionErrorIntegral,
                                                        V_IntakePID_Gain[E_PID_Proportional],
                                                        V_IntakePID_Gain[E_PID_Integral],
                                                        V_IntakePID_Gain[E_PID_Derivative],
                                                        K_Intake_PID_Limit[E_PID_Proportional],
                                                       -K_Intake_PID_Limit[E_PID_Proportional],
                                                        K_Intake_PID_Limit[E_PID_Integral],
                                                       -K_Intake_PID_Limit[E_PID_Integral],
                                                        K_Intake_PID_Limit[E_PID_Derivative],
                                                       -K_Intake_PID_Limit[E_PID_Derivative],
                                                        K_IntakeCmndLimit,
                                                       -K_IntakeCmndLimit);

  V_RobotMotorCmndPct[E_RobotMotorLift] = LiftCmdDisable(V_IntakePosition,
                                                         V_IntakeLiftHeightDesired,
                                                         K_IntakeMinCmndHeight,
                                                         V_RobotMotorCmndPct[E_RobotMotorLift]);

  return (L_ControlComplete);
  }

/******************************************************************************
 * Function:     CntrlAutonOpenLoopTimer
 *
 * Description:  Control actuators that don't have feedback with an open loop timer.
 *
 ******************************************************************************/
bool CntrlAutonOpenLoopTimer(T_Actuator       L_CntrlActuator,
                             double           L_AutonTarget)
  {
  bool        L_ControlComplete = false;

  if ((L_CntrlActuator == E_ActuatorArmAngDwn) ||
      (L_CntrlActuator == E_ActuatorArmAngUp))
    {
    V_IntakeArmTimer += C_ExeTime;

    if (V_IntakeArmTimer < L_AutonTarget)
      {
      if (L_CntrlActuator == E_ActuatorArmAngDwn)
        {
        V_RobotMotorCmndPct[E_RobotMotorIntakeArmAng] = -K_AutonIntakeAngleCmnd;
        }
      else
        {
        V_RobotMotorCmndPct[E_RobotMotorIntakeArmAng] = K_AutonIntakeAngleCmnd;
        }
      }
    else
      {
      V_IntakeArmTimer = L_AutonTarget;

      V_RobotMotorCmndPct[E_RobotMotorIntakeArmAng] = 0.0;

      L_ControlComplete = true;
      }

    }
  else if (L_CntrlActuator == E_ActuatorRollers)
    {
    V_RollerTimer += C_ExeTime;

    if (V_RollerTimer < L_AutonTarget)
      {
      V_RobotMotorCmndPct[E_RobotMotorIntakeRoller] = -K_IntakeRollers; // Negative to eject cube
      }
    else
      {
      V_RollerTimer = L_AutonTarget;

      V_RobotMotorCmndPct[E_RobotMotorIntakeRoller] = 0.0;

      L_ControlComplete = true;
      }
    }

  V_RobotMotorCmndPct[E_RobotMotorIntakeArmAng] = V_RobotUserCmndPct[E_RobotUserCmndIntakeArmAng];

  V_RobotMotorCmndPct[E_RobotMotorIntakeRoller] = V_RobotUserCmndPct[E_RobotUserCmndIntakeRoller];

  L_ControlComplete = DtrmnActuatorComplete( L_AutonTarget,
                                             V_IntakePosition,
                                             K_AutonIntakeDistanceDeadband,
                                            &V_AutonIntakeLiftDebounceTimer,
                                             K_AutonDebounceThreshold);

  V_IntakeLiftHeightDesired = CntrlAutonDesiredSpeed(L_AutonTarget,
                                                     V_IntakePosition,
                                                     V_IntakeLiftHeightDesired,
                                                     K_AutonIntakeDistanceToSlow,
                                                     K_AutonIntakeMinSpeed,
                                                     K_AutonIntakeMaxSpeed,
                                                     K_AutonIntakeSpeedRamp,
                                                     V_AutonIntakeLiftDebounceTimer,
                                                     L_ControlComplete);



  return (L_ControlComplete);
  }
