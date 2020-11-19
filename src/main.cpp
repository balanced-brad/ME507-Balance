/** @file main.cpp
 *    This file defines and schedules the tasks necessary to run the term project created by 
 *    Team Balanced 2020.
 *
 *  @author  Grant Gallagher
 *  @author  Bradley Kwan
 *  @author  Zach Richter
 * 
 *  @date    01-Novemeber-2020
 */

#include <Arduino.h>
#include <PrintStream.h>
#if (defined STM32L4xx || defined STM32F4xx)
    #include <STM32FreeRTOS.h>
#endif
#include "taskshare.h"
#include "taskqueue.h"
#include <Wire.h>
#include <inclinometer.h>
#include "HardwareTimer.h"
#include <encoder_counter.h>

// Define shares and queues here

Queue<uint16_t> linearPot_queue (10);
Queue<uint16_t> encoder_queue (10);
Queue<int16_t> accelerations (4);

Share<int32_t> torque_share ("Torque");
Share<float> pwm_share ("Duty Cycle");
Share<bool> pwm_direction_share ("Motor Direction");


/** @brief   Read the current value from the IMU
 *  @details This task calls @c inclinometer()  which provides the code with the current
 *           angle of the beam relative to horizontal. 
 *           @b NOTE: The following line of code may need to be modified depending on the orientation
 *           of the IMU relative to the beam. The main fix will typically be how the @c atan2 function
 *           is used to calculate the angle of the beam.
 *  @param   p_params A pointer to function parameters which are not used.
 */
void task_IMU (void* p_params)
{
  (void) p_params;    // Does nothing but silences a compiler warning
  // When the task is initialized, it calibrates the IMU such that it reads an angle of 0 when the IMU is flat
  Wire.begin();    // Initialize I2C bus on Nucleo
  const int MPU_ADDR = 0x68; // I2C address of the MPU-6050. 
  Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  // Calibration Sequence [COMMENT OUT IF THE IMU IS ALREADY CALIBRATED]
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
  Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(MPU_ADDR, 7*2, true); // request a total of 7*2=14 registers
  int16_t calib_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  int16_t calib_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  int16_t calib_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
  calib_z = 7000 - calib_z;  // z offset so that the accelerometer is set to read at 0 degrees when flat
  calib_y = 0 - calib_y;  // y offset so that the accelerometer is set to read at 0 degrees when flat

  // This loop will continuously measure the accelerometer values coming from the MPU 6050.
  for(;;)
  {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
    Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
    Wire.requestFrom(MPU_ADDR, 7*2, true); // request a total of 7*2=14 registers
    int16_t accelerometer_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
    int16_t accelerometer_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
    int16_t accelerometer_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
    accelerations.butt_in (accelerometer_z);
    accelerations.butt_in (accelerometer_y);
    // Serial << endl << "Y accel: " << accelerometer_y<< " || Z accel: " << accelerometer_z << endl; 
    // Serial << endl << "Y accel: " << calib_y << " || Z accel: " << calib_z << endl; 
    vTaskDelay(100);
  }
}

/** @brief   Implements full-state feedback to determine the required torque to center the ball
 *  @details This task takes in readings from the encoder and the linear potentiometer to determine the states of the
 *           system: r, r_dot, theta, theta_dot. Through these values, they can be multiplied against the caclulated gain matrix
 *           to determine the necessary torque to position the ball at the center of the beam.
 * *         The state machine has the following states:
 *           * 0: Calibration
 *           * 1: Active Control
 *  @param   p_params A pointer to function parameters which are not used.
 */
void task_stateController (void* p_params)
{
  (void) p_params;                    // Does nothing but silences a compiler warning
  int16_t yaccel, zaccel, beam_angle; // values that hold the current y acceleration, z acceleration and beam angle.
  uint8_t state = 0;                  // State of the state machine
  uint16_t linpot_pos_old, linpot_pos_new, linpot_vel;
  uint16_t state_r, r_dot, theta, theta_dot;
  float controller_period = 60;
  
  for (;;)
  {
    // The state machine runs insde the for-ever loop
    if (state == 0)          // Calibration sequence
    {  
      // Read current accelerometer values to get angle of beam
      accelerations.get(yaccel);
      accelerations.get(zaccel);
      beam_angle = incline_angle(yaccel,zaccel);
      pwm_share.put(30);     // Make motor move slowly as it tries to find an angle close to 0 degrees. 
      if (beam_angle >= abs(1))   // If beam is +/- 1 degree relative to horizontal, begin control of beam.
      {
        state = 1;
        // Insert code here that sets encoder values to zero
      }
    }
    else if (state == 1)
    {
      // Insert code here to read current position of encoder
      // From here also calculate the velocity of the motor based on timing of encoder task
      
      // Calculate states for the radial direction
      linearPot_queue.get(linpot_pos_new);   // Pull the newest value from the queue first
      linearPot_queue.get(linpot_pos_old);   // Pull the second newest value from the queue
      state_r = linpot_pos_new;              // The first pulled value is the most recently updated position of the ball.
      // Dividing the positions by the period of the task yields the average velocity of the ball.
      r_dot = (linpot_pos_new-linpot_pos_old)/(controller_period);

      // Insert Control Algorithm Code here
    }

  }
}

/** @brief   Reads current encoder values from the DC motor
 *  @details The purpose of this task is to continuously read values from the encoder and write them into a queue such that the controller task
 *           aware of the current position of the motor as well as the velocity of the motor based on the last reading. Note: this function users timer 3
 *           channels 1 and 2. Although there are other pins on the ST32, it seems that pin PA6 and PA7 cannot be used to also initialize timer 3.
 *  @param   p_params A point to function parameters which are not used.
 */

void task_encoder (void* p_params)
{
  (void) p_params;    // Does nothing but silences a compiler warning
  STM32Encoder timer_3 (TIM3, PB4, PB5);
  int16_t curr_pos;
  for(;;) 
  {
    curr_pos = timer_3.getCount(); // Get current position from encoder count
    linearPot_queue.put(curr_pos); // Place current position into queue
  }
}

/** @brief   Determines the correct PWM output based on the desired torque
 *  @details The following task calculates the PWM value necessary to output the torque (from @c torque_share ) that is
 *           calculated by the state controller task. Every time the task runs, it writes the necessary
 *           PWM value into @c pwm_share.
 *  @param   p_params A pointer to function parameters which are not used.
 */
void task_motorControl (void* p_params)    // ZACH's to relate torque to PWM
{
  (void) p_params;    // Does nothing but silences a compiler warning
}


/** @brief   Outputs appropriate PWM value to motor
 *  @details The following task reads from @c pwm_share to write the correct duty cycle value to the motor driver chip.
 *           Additionally, the direction of the motor is determined from @c pwm_direction_share which is sent to the 
 *           PH pin of the motor driver chip. In short, one pin receives the PWM signal (always positive) while the other
 *           pin determines the direction of the motor based on logic HIGH/LOW.
 *  @param   p_params A point to function parameters which are not used.
 */
void task_motorDriver (void* p_params)
{
  (void) p_params;    // Does nothing but silences a compiler warning
  float duty_cycle_var;
  bool motor_direction;
  // Initialize NSLEEP Pin of DRV8876 Motor Driver Chip
  pinMode(PA9, OUTPUT);
  digitalWrite(PA9, HIGH);
  // Initialize EN Pin of DRV8876 Motor Driver Chip (PWM Written to this Pin)
  pinMode(PB10, OUTPUT);
  // Initailize PH Pin of DRV8876 Motor Driver Chip (Direction Pin)
  pinMode(PA8, OUTPUT);

  for (;;)
  {
    pwm_share.get(duty_cycle_var);            // Duty cycle provided is always a positive value
    pwm_direction_share.get(motor_direction); // This bool determines the direction which the motor should be driven in.
    digitalWrite(PA8, motor_direction);      // Set direction of motor
    analogWrite(PB10, duty_cycle_var);      // Run motor at this duty cycle
  }
}

/** @brief   Reads value from Linear Potentiometer
 *  @details The following task reads the current signal value from the linear potentiometer. The values will
 *           then be written into a queue such that the current position and average velocity can be found.
 *  @param   p_params A point to function parameters which are not used.
 */
void task_linearpot (void* p_params)
{
  (void) p_params; // Mute compiler warning for p_params.
  // Set up pin A1 as an input, so that the signal from the linear potentiometer can be read.
  pinMode(PA1, INPUT);
  uint16_t linpot_reading;

  for(;;)
  {
    linpot_reading = analogRead(PA1);       // Read current position of the bearing on the rail
    linearPot_queue.butt_in(linpot_reading); // Store newest value of position at the front of the queue
  }
}


void setup() {
  // Start the Serial Port
  Serial.begin(115200);
  
  // Create Linear Potentiometer Task
  xTaskCreate (task_linearpot,
               "Beam Position",
              1024,
              NULL,
              4,
              NULL);
  
  // Create Motor Driver Task
  xTaskCreate (task_motorDriver,
               "Motor Driver",
               1024,
               NULL,
               3,
               NULL);

  // Create Encoder Task
  xTaskCreate (task_encoder,
               "Motor Position",
               1536,
               NULL,
               4,
               NULL);
  

  // Create IMU task
  xTaskCreate (task_IMU,
               "Beam Angle",
               1024,
               NULL,
               4,
               NULL);

  // Create State Controller Task
  xTaskCreate (task_stateController,
               "Full State Feedback Control",
               1536,
               NULL,
               2,
               NULL);
  
  // Create Motor/Current Controller Task
  xTaskCreate (task_motorControl,
               "Current/Torque Control",
               1536,
               NULL,
               2,
               NULL);
  vTaskStartScheduler ();
}

void loop() {
  // put your main code here, to run repeatedly:
}