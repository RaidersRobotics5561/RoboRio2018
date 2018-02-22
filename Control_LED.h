/*
  Control_LED.h

   Created on: Feb 20, 2018
       Author: sdbig
 */
#ifndef SRC_ROBORIO2018_CONTROL_LED_H_
#define SRC_ROBORIO2018_CONTROL_LED_H_

LED_Mode UpdateLED_Output(RoboState      L_RobotState,
                          bool           L_DriverOverride,
                          double         L_Winch,
                          DigitalOutput *L_LED_State0,
                          DigitalOutput *L_LED_State1,
                          DigitalOutput *L_LED_State2,
                          DigitalOutput *L_LED_State3);



#endif /* SRC_ROBORIO2018_CONTROL_LED_H_ */
