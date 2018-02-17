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

const double K_IntakeRollers        =  0.6;
const double K_EndMatchWarningTime  =  30;    // This is the expected time remaining that the robot will warn the
const double K_WinchOnThreshold     =   0.1; // Threshold above which the winch is considered to be on.
const double K_LED_WinchOnTime      =   3.0;  // This is the amount of accumulated time that the winch needs to be commanded on at the end of the game in order to trigger the rainbow effect

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

const double K_DesiredDriveSpeed[20] = {-200.0,  //-0.95
                                        -190.0,  //-0.85
                                        -120.0,  //-0.75
                                         -70.0,  //-0.65
                                         -50.0,  //-0.55
                                         -30.0,  //-0.45
                                          -20.0,  //-0.35
                                          -10.0,  //-0.25
                                          -5.0,  //-0.15
                                           0.0,  //-0.10
                                           0.0,  // 0.10
                                           5.0,  // 0.15
                                           10.0,  // 0.25
                                           20.0,  // 0.35
                                          30.0,  // 0.45
                                          50.0,  // 0.55
                                          70.0,  // 0.65
                                          120.0,  // 0.75
                                         190.0,  // 0.85
                                         200.0}; // 0.95

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
