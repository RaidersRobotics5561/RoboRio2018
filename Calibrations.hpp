/*
  Calibrations.hpp

   Created on: Feb 14, 2018
       Author: 5561
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
const double K_MaxIntakeLiftHeight = 52; // Max lift height in inches...
const double K_LowerIntakeLiftHeight = 1;
const double K_IntakeLiftLagFilter = 0.8;
const double K_Intake_PID_Gain[E_PID_Sz] =
    // P        I      D
    { 0.3, 0.009, 0.0 };
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
const double K_HookCmndLimit = 0.7;

const double K_Winch = 1.0;
const double K_LukeStopperRamp = 5561;

const double K_RotateGain = 0.75;
const double K_JoystickAnalogDeadband    = 0.1;

const double C_WheelSpeedPID_Gain[E_RobotSideSz][E_PID_Sz] = {
    // P    I    D
		{ 0.002,   0.0005,  0.0 }, //LEFT 002  0005
	    { 0.0014,  0.00045, 0.0 }}; // Other Left
//    { 0.0001, 0.0006, 0.0 }, //LEFT
//    { 0.0002, 0.0009, 0.0 }}; //RIGHT

const double K_WheelSpeedPID_GainAuton[E_RobotSideSz][E_PID_Sz] = {
    // P    I    D
		{ 0.0001, 0.0006, 0.0 }, //LEFT
	    { 0.0002, 0.0009, 0.0 }};
//    { 0.002,   0.0005,  0.0 }, //LEFT 002  0005
//    { 0.0014,  0.00045, 0.0 }}; //RIGHT 0015  00045

const double C_WheelspeedProportionalLimit[E_RobotSideSz][E_IntergalLimitSz] = {
    // UPPER LOWER
    { 0.7, -0.7}, //LEFT
    { 0.7, -0.7 }  //RIGHT
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

const double K_UltraSonicLagFilterGain[E_RobotSideSz] =
    { 0.6, 0.6};

const double K_UltraMaxDistance = 20.0;

const double K_IntakeRollers        =   1.0;
const double K_EndMatchWarningTime  =  30.0; // This is the expected time remaining that the robot will warn the
const double K_WinchOnThreshold     =   0.1; // Threshold above which the winch is considered to be on.
const double K_LED_WinchOnTime      =   3.0; // This is the amount of accumulated time that the winch needs to be commanded on at the end of the game in order to trigger the rainbow effect

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

const double K_DesiredDriveSpeed[20] = {-150.00,  //-0.95
                                        -131.25,  //-0.85
                                        -112.50,  //-0.75
                                         -93.75,  //-0.65
                                         -75.00,  //-0.55
                                         -56.25,  //-0.45
                                         -37.50,  //-0.35
                                         -18.75,  //-0.25
                                          -5.00,  //-0.15
                                           0.00,  //-0.10
                                           0.00,  // 0.10
                                           5.00,  // 0.15
                                          18.75,  // 0.25
                                          37.50,  // 0.35
                                          56.25,  // 0.45
                                          75.00,  // 0.55
                                          93.75,  // 0.65
                                         112.50,  // 0.75
                                         131.25,  // 0.85
                                         150.00}; // 0.95

//2.0,  // 0.15
//5.0,  // 0.25
//10.0,  // 0.35
//15.0,  // 0.45
//25.0,  // 0.55
//35.0,  // 0.65
//60.0,  // 0.75
//80.0,  // 0.85
//150.0}; // 0.95

const double K_AutonDebounceThreshold     =  0.05;   // Seconds (not sure if this is sufficient to share across all functions, but let's start with a single place value).

const double K_AutonRotateAnglePropGx     =   0.65561;  // RPM/Degrees
const double K_AutonRotateAngleDeadband   =   2.0;  // Degrees
const double K_AutonRotateMaxSpeed        =  34.5561;  // RPM
const double K_AutonRotateMinSpeed        =  11.5561;  // RPM

const double K_AutonDriveDistanceUltraDeadband =  0.5;  // Inches - for the ultrasonic sensor control
const double K_AutonDriveMinSpeedUltra         =  8.0; // RPM

const double K_AutonDriveDistanceDeadband =   5.0;  // Inches
const double K_AutonDriveDistanceToSlow   =  24.5561;  // Inches 85
const double K_AutonDriveSpeedRamp        = 100.0;  // RPM/Sec
const double K_AutonDriveMaxSpeed         = 139.5561;  // RPM
const double K_AutonDriveMinSpeed         =  44.5561;  // RPM

const double K_AutonIntakeRamp             =  60.0;  // Inch/Sec
const double K_AutonIntakeDistanceDeadband =   2.0;  // Inches

const double K_AutonIntakeAngleCmnd        =   0.6;  // Percent of possible motor controller output

/* K_AutonCommand: This table contains all of the possible commands that could be expected in Auton.
                   These will go sequentually go through the possible combinations.  You can have two
                   actuators beign controlled at once when it makes sense.  For example, you can only
                   control the drive wheels with a single command at one time (so you couldn't
                   command a roation and a drive command at the same time). The following are the
                   units for the various actuators:
                   Encoder: Inches
                   ArmAng: Open loop power command for the given time
                   Rollers: Open loop power command for the given time
                   Lift: Inches
                   Rotate: Degrees*/
const AutonControlPlan K_AutonCommand[E_AutonOptSz] =
    {
      { // E_AutonOpt0 - Left- side switch - left
        {   E_ActuatorDriveEncoder,          E_ActuatorRotate,    E_ActuatorDriveEncoder, E_ActuatorDriveUltraSonic,         E_ActuatorRollers,    E_ActuatorDriveEncoder,          E_ActuatorRotate,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone},
        {                      160,                        90,                        20,                         7,                       1.5,                        -5,                       -90,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0},
        {           E_ActuatorNone,            E_ActuatorLift,            E_ActuatorNone,       E_ActuatorArmAngDwn,            E_ActuatorNone,        E_ActuatorArmAngUp,            E_ActuatorLift,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone},
        {                        0,                        30,                         0,                       0.6,                         0,                       0.6,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0}
      },
      { // E_AutonOpt1 - Left - Scale - Left
        {   E_ActuatorDriveEncoder,          E_ActuatorRotate,    E_ActuatorDriveEncoder,       E_ActuatorArmAngDwn,         E_ActuatorRollers,        E_ActuatorArmAngUp,          E_ActuatorRotate,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone},
        {                      300,                        90,                         -5,                      0.6,                       1.5,                       0.6,                       -90,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0},
        {           E_ActuatorNone,            E_ActuatorNone,            E_ActuatorLift,            E_ActuatorNone,            E_ActuatorNone,    E_ActuatorDriveEncoder,            E_ActuatorLift,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone},
        {                        0,                         0,                        52,                         0,                         0,                       -10,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0}
      },
      { // E_AutonOpt2 - middle - front switch - left
        {   E_ActuatorDriveEncoder,          E_ActuatorRotate,    E_ActuatorDriveEncoder,          E_ActuatorRotate,   				 E_ActuatorNone, 				E_ActuatorNone,       E_ActuatorDriveUltraSonic,         E_ActuatorNone,        E_ActuatorNone,          E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone},
        {                      14.5561,                       -54.5561,                        130,                      54.5561,                        0,                        10,                       6.5561,                       1.5,                         1,                       -90,                         0,                         0,                         0,                         0,                         0},
        {           E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,    E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone},
        {                        0,                         0,                         0,                         0,                         0,                        30,                         0,                         0,                        -5,                         0,                         0,                         0,                         0,                         0,                         0}
      },
      { // E_AutonOpt3 - middle - front switch - right
        {   E_ActuatorNone,          E_ActuatorRotate,    E_ActuatorDriveEncoder,          E_ActuatorNone,   			E_ActuatorNone,				E_ActuatorDriveUltraSonic,				E_ActuatorNone,         E_ActuatorNone,        E_ActuatorNone,          E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone},
        {                       0,                       5,                        75,                       0,                        0,                        6.5561,                       0,                       0,                         0,                        0,                         0,                         0,                         0,                         0,                         0},
        {           E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,    	E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone},
        {                        0,                         0,                         0,                         0,                        0,                         0,                         0,                         0,                        0,                         0,                         0,                         0,                         0,                         0,                         0}
      },
      { // E_AutonOpt4 - Right - Scale - Right
        {   E_ActuatorDriveEncoder,          E_ActuatorRotate,    E_ActuatorDriveEncoder,       E_ActuatorArmAngDwn,         E_ActuatorRollers,        E_ActuatorArmAngUp,          E_ActuatorRotate,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone},
        {                      300,                       -90,                        -5,                       0.6,                       1.5,                       0.6,                        90,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0},
        {           E_ActuatorNone,            E_ActuatorNone,            E_ActuatorLift,            E_ActuatorNone,            E_ActuatorNone,    E_ActuatorDriveEncoder,            E_ActuatorLift,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone},
        {                        0,                         0,                        52,                         0,                         0,                       -10,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0}
      },
      { // E_AutonOpt5 - right - switch - right
        {   E_ActuatorDriveEncoder,          E_ActuatorRotate,    E_ActuatorDriveEncoder, E_ActuatorDriveUltraSonic,         E_ActuatorRollers,    E_ActuatorDriveEncoder,          E_ActuatorRotate,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone},
        {                      160,                       -90,                        20,                        7,                       1.5,                        -5,                        90,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0},
        {           E_ActuatorNone,            E_ActuatorLift,            E_ActuatorNone,       E_ActuatorArmAngDwn,            E_ActuatorNone,        E_ActuatorArmAngUp,            E_ActuatorLift,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone},
        {                        0,                        30,                         0,                       0.6,                         0,                       0.6,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0}
      },
      { // E_AutonOpt6 - Default
        {   E_ActuatorDriveEncoder,            E_ActuatorNone,       E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone},
        {                      120,                         0,                       0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0},
        {           E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone},
        {                        0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0}
      }
    };




#endif /* SRC_ROBORIO2018_CALIBRATIONS_HPP_ */
