/*
  Enums.hpp

   Created on: Feb 14, 2018
       Author: sdbig
 */
#ifndef SRC_ROBORIO2018_ENUMS_HPP_
#define SRC_ROBORIO2018_ENUMS_HPP_

 typedef enum
 {
   C_BlinkR,
   C_BlinkL,
   C_BlinkSz
 }Blinker;

 typedef enum
 {
   LED_Color_Red,
   LED_Color_Blue,
   LED_Color_Green,
   LED_Color_White,
   LED_Color_Purple,
   LED_Color_Yellow,
   LED_Color_Pink,
   LED_Color_Orange,
   LED_Color_Rainbow, // This is meant to indicate when a random mixture of colors are desired
   LED_Color_Multi,   // This is meant to indicate when it is desired to have the colors cycle through all of the avaiable colors above rainbow
   LED_Color_Black    // This is more of an "off mode", must remain at end of enum
 } LED_Color;

 typedef enum
 {
   LED_Mode0,
   LED_Mode1,
   LED_Mode2,
   LED_Mode3,
   LED_Mode4,
   LED_Mode5,
   LED_Mode6,
   LED_Mode7,
   LED_Mode8,
   LED_Mode9,
   LED_Mode10,
   LED_Mode11,
   LED_Mode12,
   LED_Mode13,
   LED_Mode14,
   LED_Mode15,
   LED_ModeSz
 } LED_Mode;

 typedef enum
 {
   C_Disabled,
   C_Auton,
   C_Teleop,
   C_Test,
   C_Null
 }RoboState;

typedef enum {
  E_RobotSideLeft, E_RobotSideRight, E_RobotSideSz
} T_RobotSide;

typedef enum {
  E_PID_Proportional, E_PID_Integral, E_PID_Derivative, E_PID_Sz
} T_PID;

typedef enum {
  E_IntergalUpperLimit, E_IntergalLowerLimit, E_IntergalLimitSz
} T_IntergalLimit;



#endif /* SRC_ROBORIO2018_ENUMS_HPP_ */
