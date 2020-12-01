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

// Define task timings here
float encoder_timing = 5;
float linear_pot_timing = 5;

// Define shares and queues here

Queue<float> linearPot_queue (5);
Queue<float> encoder_queue (5);
Queue<int16_t> accelerations (4);

Share<float> duty_cycle_share ("Duty Cycle");
Share<bool> mot_dir_share ("Motor Direction");


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
  // calib_z = 7000 - calib_z;  // z offset so that the accelerometer is set to read at 0 degrees when flat
  // calib_y = 0 - calib_y;  // y offset so that the accelerometer is set to read at 0 degrees when flat

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
    vTaskDelay(5);
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
  uint8_t state = 1;                  // State of the state machine
  float linpot_pos_old, linpot_pos_new;  // Initialize values for linear potentiometer readings
  float encoder_old, encoder_new, encoder_calib;        // Initialize values for encoder tick readings and calibration values
  float stateR, r_dot, theta, theta_dot;
  float controller_period = 12;

  int8_t K1 = -78;    //Gain matrix element 1
  int8_t K2 = -140;    //Gain matrix element 2
  int16_t K3 = 899;   //Gain matrix element 3
  int8_t K4 = 86;     //Gain matrix element 4

  float Kp_gain_motor = 100;  // Gain of the motor controller 
  uint16_t R_IPROPI = 1500;      // Resistance value for current
  float V_ipropi;
  float Kt_motor = 175.5;        // motor torque constant [ozf-in/A]
  float I_ipropi;                // current in uA
  float I_motor;                 // current from the motor driver [A]
  float torque_desired;          // torque desired from the motor [lbm-in^2/s^2] from feedback gain matrix
  float torque_actual;           // actual torque of the motor [lbm-in^2/s^2]
  float pwm_change;  
  float pwm_new = 0;

  //torque_desired = K1*stateR + K2*r_dot + K3*theta + K4*theta_dot;

  //Set up IPROPI pin for analog input
  pinMode(PA_0,INPUT);
  
  for (;;)
  {
    // The state machine runs insde the for-ever loop
    if (state == 0)          // Calibration sequence
    {  
      // Read current accelerometer values to get angle of beam
      accelerations.get(yaccel);
      accelerations.get(zaccel);
      beam_angle = incline_angle(yaccel,zaccel);
      duty_cycle_share.put(50);     // Make motor move slowly as it tries to find an angle close to 0 degrees.  [May need to adjust value to make it go slower]
      if (abs(beam_angle) <= 1)   // If beam is +/- 1 degree relative to horizontal, begin control of beam.
      {
        state = 1;  // Transition to control state
        encoder_queue.get(encoder_calib);     // Set calibration value of encoder such that encoder ticks read at zero when the beam is flat.
      }
      vTaskDelay(controller_period);
    }
    else if (state == 1)
    {
      // Calculate states for the radial direction
      linearPot_queue.get(linpot_pos_new);   // Pull the newest value from the queue first
      linearPot_queue.get(linpot_pos_old);   // Pull the second newest value from the queue
      stateR = linpot_pos_new;              // The first pulled value is the most recently updated position of the ball.
      // Dividing the positions by the period of the task yields the average velocity of the ball.
      r_dot = 1000 * (linpot_pos_new-linpot_pos_old)/(linear_pot_timing);
      //Serial << endl << endl << "r_dot: " << r_dot << endl;
      //Serial << endl << "linpot_pos_old: " << linpot_pos_old << endl;
      //Serial << endl << "linpot_pos_new: " << linpot_pos_new << endl;
      // Calculate states for the theta directions
      encoder_queue.get(encoder_new);     // Pull the newest value from the queue first
      encoder_new -= encoder_calib;       // Adjust new encoder reading with calibration value
      encoder_queue.get(encoder_old);     // Pull the second newest value from the queue
      encoder_old -= encoder_calib;       // Adjust old encoder reading with calibration value
      theta = encoder_new;                // The first pulled value is the most recently updated angle [rad]
      // Dividing the encoder values by the period of the task yields the angular velocity of the motor.
      theta_dot = 1000 *(encoder_new-encoder_old)/encoder_timing; 
      //Serial << endl << endl << "theta_dot: " << theta_dot << endl;

      //Calculate the desired torque from the feedback gain matrix
      torque_desired = K1*stateR + K2*r_dot + K3*theta + K4*theta_dot;
      Serial << endl << endl << "torque_desired : " << torque_desired << endl;
      //Calculate the current being supplied to the motor
      V_ipropi = analogRead(PA_0);
      I_ipropi = V_ipropi/R_IPROPI; 
      I_motor = I_ipropi;
      //Calculate the torque that the motor is currently applying and compare to desired
      torque_actual = I_motor*Kt_motor*(24.15);
      Serial << endl << endl << "torque_actual : " << torque_actual << endl;

      //Determine pwm adjustment for any error between torque
      pwm_change = (torque_desired - torque_actual)*Kp_gain_motor;
      pwm_new -= pwm_change;
      
      //see if pwm value is positive or negative
      if (pwm_new < 0)
      {
        //pwm max absolute value is 100
        if(pwm_new < -100)
        {
          pwm_new = -100;
        }
        duty_cycle_share.put(-pwm_new);      //put negative of pwm value and set direction bool (unsure if this sytax works)
        mot_dir_share.put(HIGH); 
      }
      else
      {
        //pwm max absolute value is 100
        if(pwm_new > 100)
        {
          pwm_new = 100;
        }
        duty_cycle_share.put(pwm_new);
        mot_dir_share.put(LOW);
      } 
      vTaskDelay(controller_period);
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
  STM32Encoder timer_3 (TIM3, PB4, PB5);  // Initialize timer 3 channel 1 and channel 2 in encoder mode
  int16_t curr_pos;        // Tick reading from encoder
  const float PPR = 360*4;   // Define number of pulses per revolution for DC motor (multiplied by 4 to account for 2 channels reading in encoder mode)
  for(;;) 
  {
    curr_pos = timer_3.getCount(); // Get current position from encoder count
    curr_pos = curr_pos/(PPR)*2*PI;  // Normalize encoder tick reading and then multiply by 2PI to find position in terms of rads
    encoder_queue.put(curr_pos); // Place current position [rad] into queue

    vTaskDelay(encoder_timing);
  }
}

/** @brief   Outputs appropriate PWM value to motor
 *  @details The following task reads from @c duty_cycle_share to write the correct duty cycle value to the motor driver chip.
 *           Additionally, the direction of the motor is determined from @c mot_dir_share which is sent to the 
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
    duty_cycle_share.get(duty_cycle_var);            // Duty cycle provided is always a positive value
    mot_dir_share.get(motor_direction); // This bool determines the direction which the motor should be driven in.
    digitalWrite(PA8, motor_direction);       // Set direction of motor
    analogWrite(PB10, duty_cycle_var);        // Run motor at this duty cycle
    vTaskDelay(12);
  }
}

/** @brief   Reads value from Linear Potentiometer
 *  @details The following task reads the voltage reading from the linear potentiometer. The value is then
 *           normalized and then written into a queue such that the current position and average velocity can be found.
 * 
 *           Note: @c analogRead() provides 0 - 1023 reading from 0 - 3.3 V
 *  @param   p_params A point to function parameters which are not used.
 */
void task_linearpot (void* p_params)
{
  (void) p_params; // Mute compiler warning for p_params.
  // Set up pin A1 as an input, so that the signal from the linear potentiometer can be read.
  pinMode(PA1, INPUT);
  float linpot_reading;      // Initialize analog voltage reading 
  float linpot_reading_inch;  // Initialize inch calculation for ball position.

  for(;;)
  {
    linpot_reading = analogRead(PA1);           // Read voltage reading from linear potentiometer
    //Serial << endl << endl << "Linear Pot Reading Raw: " << linpot_reading << endl;

    linpot_reading_inch = ((linpot_reading/1023) - 0.07)*12 - 6;    // Normalize the voltage value and multiply by the length of the beam. (Note: 0 in. actually signifies the center of the beam)
    //Serial << endl << endl << "Linear Pot Reading Cal             " << linpot_reading_inch << endl;
    linearPot_queue.butt_in(linpot_reading_inch);    // Store position of the ball on the beam into the front of the queue.
    vTaskDelay(linear_pot_timing);
  }
}


void setup() {
  // Start the Serial Port
  Serial.begin(115200);
  delay(2000);
  
  // Create Linear Potentiometer Task
  xTaskCreate (task_linearpot,
               "Beam Position",
              1024,
              NULL,
              3,
              NULL);
  
  // Create Motor Driver Task
  xTaskCreate (task_motorDriver,
               "Motor Driver",
               1024,
               NULL,
               2,
               NULL);

  // Create Encoder Task
  xTaskCreate (task_encoder,
               "Motor Position",
               1536,
               NULL,
               3,
               NULL);
  

  // Create IMU task
  xTaskCreate (task_IMU,
               "Beam Angle",
               1024,
               NULL,
               3,
               NULL);

  // Create State Controller Task
  xTaskCreate (task_stateController,
               "Full State Feedback Control",
               1536,
               NULL,
               5,
               NULL);
  
  vTaskStartScheduler ();
}

void loop() {
  // put your main code here, to run repeatedly:
}