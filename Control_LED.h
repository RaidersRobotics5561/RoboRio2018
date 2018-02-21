/*
  Control_LED.h

   Created on: Feb 20, 2018
       Author: sdbig
 */
#ifndef SRC_ROBORIO2018_CONTROL_LED_H_
#define SRC_ROBORIO2018_CONTROL_LED_H_

LED_Mode UpdateLED_Output(RoboState L_RobotState,
                          bool      L_DriverOverride,
                          double    L_Winch,
                          bool      *L_LED_CmndState);



#endif /* SRC_ROBORIO2018_CONTROL_LED_H_ */
