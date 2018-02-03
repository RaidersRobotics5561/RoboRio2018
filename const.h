/*
 * const.h
 *
 *  Created on: Feb 1, 2018
 *      Author: wesat
 */

#ifndef SRC_CONST_H_
#define SRC_CONST_H_

const double C_speedGain = 10;
const double C_SpeedFilterGain = 0.01;

const double C_ErrorP_L = 0.06;
const double C_ErrorI_L = 0.01;
const double C_ErrorD_L = 0.01;
const double C_IntergalUpperLimit_L = 0.1;
const double C_IntergalLowerLimit_L = -0.1;
const double C_FiltGain_L = 0.01;

const double C_ErrorP_R = 0.06;
const double C_ErrorI_R = 0.01;
const double C_ErrorD_R = 0.01;
const double C_IntergalUpperLimit_R = 0.1;
const double C_IntergalLowerLimit_R = -0.1;
const double C_FiltGain_R = 0.01;

const double C_ExeTime = 0.01;

#endif /* SRC_CONST_H_ */
