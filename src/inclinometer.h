/** @file   inclinometer.h
 *          This file contains the header for the inclinometer.h function.
 *  @author Bradley Kwan
 *  @date   2020-31-Oct
 */

#include <Arduino.h>
#include <PrintStream.h>

// Function that takes two acceleration readings and calculates angle.
int16_t incline_angle (int16_t yaccel, int16_t zaccel);