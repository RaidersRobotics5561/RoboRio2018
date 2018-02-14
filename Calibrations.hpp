/*
  Calibrations.hpp

   Created on: Feb 14, 2018
       Author: sdbig
 */
#ifndef SRC_ROBORIO2018_CALIBRATIONS_HPP_
#define SRC_ROBORIO2018_CALIBRATIONS_HPP_



const double C_WheelSpeedPID_Gain[E_RobotSideSz][E_PID_Sz] = {
    // P    I    D
    { 0.01, 0.001, 0.0 }, //LEFT
    { 0.01, 0.001, 0.0 }}; //RIGHT

const double C_WheelspeedProportionalLimit[E_RobotSideSz][E_IntergalLimitSz] = {
    // UPPER LOWER
    { 1.0, -1.0}, //LEFT
    { 1.0, -1.0}  //RIGHT
};

const double C_WheelspeedIntergalLimit[E_RobotSideSz][E_IntergalLimitSz] = {
    // UPPER LOWER
    { 0.6, -0.6}, //LEFT
    { 0.6, -0.6}  //RIGHT
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
    { 0.6, 0.6};

const double K_EndMatchWarningTime  =  30;    // This is the expected time remaining that the robot will warn the
const double K_WinchOnThreshold     =   0.1; // Threshold above which the winch is considered to be on.
const double K_LED_WinchOnTime      =   3.0;  // This is the amount of accumulated time that the winch needs to be commanded on at the end of the game in order to trigger the rainbow effect

const double K_DesiredDriveSpeedAxis[20] = {-1.00,
                                            -0.90,
                                            -0.80,
                                            -0.70,
                                            -0.60,
                                            -0.50,
                                            -0.40,
                                            -0.30,
                                            -0.20,
                                            -0.10,
                                             0.00,
                                             0.10,
                                             0.20,
                                             0.30,
                                             0.40,
                                             0.50,
                                             0.60,
                                             0.70,
                                             0.80,
                                             0.90};

const double K_DesiredDriveSpeed[20] = {-100.0,  //-1.00
                                        -50.0,  //-0.90
                                        -25.0,  //-0.80
                                        -21.0,  //-0.70
                                        -17.0,  //-0.60
                                        -13.0,  //-0.50
                                         -9.0,  //-0.40
                                         -5.0,  //-0.30
                                         -2.0,  //-0.20
                                          0.0,  //-0.10
                                          0.0,  // 0.00
                                          0.0,  // 0.10
                                          2.0,  // 0.20
                                          5.0,  // 0.30
                                          9.0,  // 0.40
                                         13.0,  // 0.50
                                         17.0,  // 0.60
                                         21.0,  // 0.70
                                         50.0,  // 0.80
                                         100.0}; // 0.90



#endif /* SRC_ROBORIO2018_CALIBRATIONS_HPP_ */
