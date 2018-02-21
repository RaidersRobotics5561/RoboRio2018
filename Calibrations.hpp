/*
  Calibrations.hpp

   Created on: Feb 14, 2018
       Author: sdbig
 */
#ifndef SRC_ROBORIO2018_CALIBRATIONS_HPP_
#define SRC_ROBORIO2018_CALIBRATIONS_HPP_

#define PracticeBot

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

const double K_IntakeRollers        =  0.8;
const double K_EndMatchWarningTime  =  30;    // This is the expected time remaining that the robot will warn the
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

const double K_DesiredVerticalSpeed[10] = {-30.0, // 0.0
                                           -15.0, // 0.1
                                           -10.0, // 0.2
                                            -2.0, // 0.3
                                             0.0, // 0.4
                                             0.0, // 0.5
                                             2.0, // 0.6
                                            10.0, // 0.7
                                            15.0, // 0.8
                                            30.0}; // 0.9

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

const double K_DesiredDriveSpeed[20] = {-150.0,  //-0.95
                                        -80.0,  //-0.85
                                        -60.0,  //-0.75
                                         -35.0,  //-0.65
                                         -25.0,  //-0.55
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
                                          25.0,  // 0.55
                                          35.0,  // 0.65
                                          60.0,  // 0.75
                                         80.0,  // 0.85
                                         150.0}; // 0.95


//const double K_DesiredDriveSpeed[20] = {-200.0,  //-0.95
//                                        -190.0,  //-0.85
//                                        -120.0,  //-0.75
//                                         -70.0,  //-0.65
//                                         -50.0,  //-0.55
//                                         -30.0,  //-0.45
//                                          -20.0,  //-0.35
//                                          -10.0,  //-0.25
//                                          -5.0,  //-0.15
//                                           0.0,  //-0.10
//                                           0.0,  // 0.10
//                                           5.0,  // 0.15
//                                           10.0,  // 0.25
//                                           20.0,  // 0.35
//                                          30.0,  // 0.45
//                                          50.0,  // 0.55
//                                          70.0,  // 0.65
//                                          120.0,  // 0.75
//                                         190.0,  // 0.85
//                                         200.0}; // 0.95

const double K_DesiredDriveSpeedSlow[20] = {-200.0,  //-0.95
                                        -95.0,  //-0.85
                                        -60.0,  //-0.75
                                         -35.0,  //-0.65
                                         -25.0,  //-0.55
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
                                          25.0,  // 0.55
                                          35.0,  // 0.65
                                          60.0,  // 0.75
                                         95.0,  // 0.85
                                         200.0}; // 0.95



#endif /* SRC_ROBORIO2018_CALIBRATIONS_HPP_ */
