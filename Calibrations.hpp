/*
  Calibrations.hpp

   Created on: Feb 14, 2018
       Author: sdbig
 */
#ifndef SRC_ROBORIO2018_CALIBRATIONS_HPP_
#define SRC_ROBORIO2018_CALIBRATIONS_HPP_

const int K_SlotIdx = 0;
const int K_PIDLoopIdx = 0;
const int K_TimeoutMs = 10;
const double K_WheelPulseToRev = 12.75;
const double K_HookPulseToRev  =  1.0;
const double K_HookRevToDistance = 0.00133;
const double K_IntakePulseToRev  =  1.0;
const double K_IntakeRevToDistance = 0.1;

const double K_IntakePulseToTravel = -0.002693208430913349; // travel (in) / pulse
const double K_MaxIntakeLiftHeight = 68; // Max lift height in inches...
const double K_LowerIntakeLiftHeight = 2;
const double K_IntakeLiftLagFilter = 0.8;
const double K_Intake_PID_Gain[E_PID_Sz] =
    // P        I      D
    { 0.0001, 0.0006, 0.0 };
const double K_Intake_PID_Limit[E_PID_Sz] =
    // P        I      D
    { 0.9, 0.9, 0.0 };
const double K_IntakeDerivativeLimit = 0.0;
const double K_IntakeCmndLimit = 1.0;
const double K_IntakeMinCmndHeight = 1.0;


const double K_HookPulseToTravel = 0.0013329068031563234; // travel (in) / pulse
const double K_MaxHookLiftHeight = 31.05; // Max lift height in inches...
const double K_Hook_PID_Gain[E_PID_Sz] =
    // P        I      D
    { 0.0002, 0.0008, 0.0 };
const double K_Hook_PID_Limit[E_PID_Sz] =
    // P        I      D
    { 0.5, 0.8, 0.0 };
const double K_HookProportionalLimit = 0.5;
const double K_HookIntergalLimit = 0.8;
const double K_HookDerivativeLimit = 0.0;
const double K_HookCmndLimit = 0.7;

const double K_Winch = 1.0;
const double K_LukeStopperRamp = 0.001;

const double K_RotateGain = 0.80;
const double K_JoystickAnalogDeadband    = 0.1;

const double C_WheelSpeedPID_Gain[E_RobotSideSz][E_PID_Sz] = {
    // P    I    D
    { 0.0001, 0.0006, 0.0 }, //LEFT
    { 0.0001, 0.0006, 0.0 }}; //RIGHT

const double C_WheelspeedProportionalLimit[E_RobotSideSz][E_IntergalLimitSz] = {
    // UPPER LOWER
    { 0.6, -0.6}, //LEFT
    { 0.6, -0.6 }  //RIGHT
};

const double C_WheelspeedIntergalLimit[E_RobotSideSz][E_IntergalLimitSz] = {
    // UPPER LOWER
    { 1.0, -1.0}, //LEFT
    { 1.0, -1.0}  //RIGHT
};

const double C_WheelspeedDerivativeLimit[E_RobotSideSz][E_IntergalLimitSz] = {
    // UPPER LOWER
    { 1.0, -1.0}, //LEFT
    { 1.0, -1.0}  //RIGHT
};

const double C_WheelspeedCmndLimit[E_RobotSideSz][E_IntergalLimitSz] = {
    // UPPER LOWER
    { 1.0, -1.0}, //LEFT
    { 1.0, -1.0}  //RIGHT
};

const double C_WheelSpeedLagFilterGain[E_RobotSideSz] =
    { 0.9, 0.9};

const double K_IntakeRollers        =   1.0;
const double K_EndMatchWarningTime  =  30.0;    // This is the expected time remaining that the robot will warn the
const double K_WinchOnThreshold     =   0.1; // Threshold above which the winch is considered to be on.
const double K_LED_WinchOnTime      =   3.0;  // This is the amount of accumulated time that the winch needs to be commanded on at the end of the game in order to trigger the rainbow effect

const double K_IntakeArmPulseToRev[E_ArmCmndSz] =
    {   0.0,                                 //   E_ArmCmndOff
        0.00557103064066852367688022284123,  //   E_ArmCmndUp
        0.00557103064066852367688022284123}; //   E_ArmCmndDwn

const double K_DesiredVerticalSpeedAxis[10] = {-0.9,
                                               -0.7,
                                               -0.5,
                                               -0.3,
                                               -0.1,
                                                0.1,
                                                0.3,
                                                0.5,
                                                0.7,
                                                0.9};

const double K_DesiredVerticalSpeed[10] = {-30.0,  // -0.9
                                           -15.0,  // -0.7
                                           -10.0,  // -0.5
                                            -2.0,  // -0.3
                                             0.0,  // -0.1
                                             0.0,  //  0.1
                                             2.0,  //  0.3
                                            10.0,  //  0.5
                                            15.0,  //  0.7
                                            30.0}; //  0.9

const double K_DesiredDriveSpeedAxis[20] = {-0.95,
                                            -0.85,
                                            -0.75,
                                            -0.65,
                                            -0.55,
                                            -0.45,
                                            -0.35,
                                            -0.25,
                                            -0.15,
                                            -0.10,
                                             0.10,
                                             0.15,
                                             0.25,
                                             0.35,
                                             0.45,
                                             0.55,
                                             0.65,
                                             0.75,
                                             0.85,
                                             0.95};

const double K_DesiredDriveSpeed[20] = {-60.0,  //-0.95
                                        -40.0,  //-0.85
                                        -30.0,  //-0.75
                                         -25.0,  //-0.65
                                         -20.0,  //-0.55
                                         -15.0,  //-0.45
                                          -10.0,  //-0.35
                                          -5.0,  //-0.25
                                          -2.0,  //-0.15
                                           0.0,  //-0.10
                                           0.0,  // 0.10
                                           2.0,  // 0.15
                                           5.0,  // 0.25
                                           10.0,  // 0.35
                                          15.0,  // 0.45
                                          20.0,  // 0.55
                                          25.0,  // 0.65
                                          30.0,  // 0.75
                                         40.0,  // 0.85
                                         60.0}; // 0.95

//2.0,  // 0.15
//5.0,  // 0.25
//10.0,  // 0.35
//15.0,  // 0.45
//25.0,  // 0.55
//35.0,  // 0.65
//60.0,  // 0.75
//80.0,  // 0.85
//150.0}; // 0.95

const double K_AutonDebounceThreshold     =  0.2;   // Seconds (not sure if this is sufficient to share across all functions, but let's start with a single place value).

const double K_AutonRotateAngleToSlow     =  15.0;  // Degrees

const double K_AutonDriveDistanceUltraDeadband = 0.5;  // Inches - for the ultrasonic sensor control
const double K_AutonDriveMinSpeedUltra         =  5.0; // RPM
const double K_AutonDriveDistanceUltraToSlow   =  2.0;  // Inches

const double K_AutonDriveDistanceDeadband =   1.0;  // Inches
const double K_AutonDriveDistanceToSlow   =  10.0;  // Inches
const double K_AutonDriveSpeedRamp        =   5.0;  // RPM/Sec
const double K_AutonDriveMaxSpeed         =  30.0;  // RPM
const double K_AutonDriveMinSpeed         =  10.0;  // RPM

const double K_AutonIntakeDistanceToSlow   =   4.0;  // Inches
const double K_AutonIntakeSpeedRamp        =   1.0;  // Inch/Sec
const double K_AutonIntakeMaxSpeed         =  20.0;  // Inch/Sec
const double K_AutonIntakeMinSpeed         =   4.0;  // Inch/Sec
const double K_AutonIntakeDistanceDeadband =   1.0;  // Inches

const double K_AutonIntakeAngleCmnd        =   0.6;  // Percent of possible motor controller output

const AutonControlPlan K_AutonCommand[E_AutonOptSz] =
    {
      { // E_AutonOpt0 - LLL
        {   E_ActuatorDriveEncoder,          E_ActuatorRotate,    E_ActuatorDriveEncoder,       E_ActuatorArmAngDwn,         E_ActuatorRollers,    E_ActuatorDriveEncoder,            E_ActuatorLift,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone},
        {                      250,                        90,                         5,                       1.2,                         2,                        -5,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0},
        {           E_ActuatorNone,            E_ActuatorLift,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,        E_ActuatorArmAngUp,          E_ActuatorRotate,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone},
        {                        0,                        30,                         0,                         0,                         0,                       1.2,                       -90,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0}
      },
      { // E_AutonOpt1 - LLR
        {           E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone},
        {                        0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0},
        {           E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone},
        {                        0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0}
      },
      { // E_AutonOpt2
        {           E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone},
        {                        0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0},
        {           E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone},
        {                        0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0}
      },
      { // E_AutonOpt3
        {           E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone},
        {                        0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0},
        {           E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone},
        {                        0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0}
      },
      { // E_AutonOpt4
        {           E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone},
        {                        0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0},
        {           E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone},
        {                        0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0}
      },
      { // E_AutonOpt5
        {           E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone},
        {                        0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0},
        {           E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone},
        {                        0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0}
      },
      { // E_AutonOpt6
        {           E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone},
        {                        0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0},
        {           E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone},
        {                        0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0}
      },
      { // E_AutonOpt7
        {           E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone},
        {                        0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0},
        {           E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone},
        {                        0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0}
      },
      { // E_AutonOpt8
        {           E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone},
        {                        0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0},
        {           E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone},
        {                        0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0}
      },
      { // E_AutonOpt9
        {           E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone},
        {                        0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0},
        {           E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone},
        {                        0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0}
      },
      { // E_AutonOpt10
        {           E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone},
        {                        0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0},
        {           E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone},
        {                        0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0}
      },
      { // E_AutonOpt11
        {           E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone},
        {                        0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0},
        {           E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone},
        {                        0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0}
      },
      { // E_AutonOpt12
        {           E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone},
        {                        0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0},
        {           E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone},
        {                        0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0}
      },
      { // E_AutonOpt13
        {           E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone},
        {                        0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0},
        {           E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone},
        {                        0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0}
      },
      { // E_AutonOpt14
        {           E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone},
        {                        0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0},
        {           E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone},
        {                        0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0}
      },
      { // E_AutonOpt15
        {           E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone},
        {                        0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0},
        {           E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone},
        {                        0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0}
      },
      { // E_AutonOpt16
        {           E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone},
        {                        0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0},
        {           E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone},
        {                        0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0}
      },
      { // E_AutonOpt17
        {           E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone},
        {                        0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0},
        {           E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone},
        {                        0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0}
      }
    };




#endif /* SRC_ROBORIO2018_CALIBRATIONS_HPP_ */
