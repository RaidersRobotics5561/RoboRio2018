/*
  SignalFilter.cpp

   Created on: Feb 14, 2018
       Author: 5561
 */



/******************************************************************************
 * Function:     LagFilter
 *
 * Description:  Simple first order lag filter.
 ******************************************************************************/
double LagFilter(double L_FilterGain,
                 double L_SpeedRaw,
                 double L_SpeedFiltPrev)
  {
  return (L_FilterGain * L_SpeedRaw + (1 - L_FilterGain) * L_SpeedFiltPrev);
  }
