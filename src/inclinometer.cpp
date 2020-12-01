/** @file   inclinometer.cpp
 *          This file contains which takes two accelerations and
 *          determines the angle of the IMU, printing the value in
 *          the serial monitor. This code will on micro-controller that
 *          accepts the Arduino framework.
 *  @author Bradley Kwan
 *  @date   2020-31-Oct
 */
#include "inclinometer.h"

/** @brief   Determines angle of IMU in the x axis.
 *  @details Performs calculation using two acceleration values to provide
 *           the angle in the x-direction ('Roll'). Note the values placed in the function
 *           may vary on mounting orientation of IMU. Future iterations can accomodate for all angles.
 *  @param   yaccel Accelerometer reading in the y-direction
 *  @param   zaccel Accelerometer reading in the z-direction 
 */
int16_t incline_angle (int16_t yaccel, int16_t zaccel)
{
    int16_t rollangle = atan2(zaccel,yaccel) * 180/PI;
    return rollangle;
}           