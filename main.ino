#include <Wire.h>                          //Include the Wire.h library so we can communicate with the gyro.
#include "MPU9250.h"

MPU9250 mpu;
MPU9250Setting setting;

// Determines change in angle
int SPEED = 4;

#define MOTOR_PIN_1 8
#define MOTOR_PIN_2 9
#define MOTOR_PIN_3 10
#define MOTOR_PIN_4 11

String tempString = "0000";
String userInputString = "0000";

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID gain and limit settings
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float pid_p_gain_roll = 1.7;               //Gain setting for the roll P-controller
float pid_i_gain_roll = 0.04; // 0.04              //Gain setting for the roll I-controller
float pid_d_gain_roll = 25;  //18.0              //Gain setting for the roll D-controller
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = 1.7;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = 0.04;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = 25;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 6.0;                //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0.02;               //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller.
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-)

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring global variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
volatile int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
int esc_1, esc_2, esc_3, esc_4;
int throttle = 1100;
float roll_level_adjust, pitch_level_adjust;

unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
unsigned long loop_timer;
double gyro_pitch, gyro_roll, gyro_yaw;
double gyro_axis_cal[4];
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup routine
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup(){
  Wire.begin();                                                             //Start the I2C as master.

  // Bluetooth Serial
  Serial1.begin(115200);

  // DEBUG
  Serial.begin(115200);

  // SETUP MOTORS
  pinMode(MOTOR_PIN_1, OUTPUT);
  analogWrite(MOTOR_PIN_1, 0);
  pinMode(MOTOR_PIN_2, OUTPUT);
  analogWrite(MOTOR_PIN_2, 0);
  pinMode(MOTOR_PIN_3, OUTPUT);
  analogWrite(MOTOR_PIN_3, 0);
  pinMode(MOTOR_PIN_4, OUTPUT);
  analogWrite(MOTOR_PIN_4, 0);
 
  // Increase sample rate
  setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_333HZ;

  // Make sure IMU plugged in
  if (!mpu.setup(0x68, setting)) {
    while (1) {
        Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
        delay(5000);
    }
  }

  // Set pre-calibrated biases
  mpu.verbose(true);
  mpu.setAccBias(25.90, 21.51, 27.62); 
  mpu.setGyroBias(0.09,1.37,0.84); 
  mpu.verbose(false);
  //print_calibration();

  // DEBUG
  Serial.println("Starting up now!"); 
  delay(2000);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main program loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop(){

  // Store Bluetooth buffer
  while (Serial1.available() > 0) {
    tempString += char(Serial1.read());
    delay(1);
  }

  if (tempString != "" && tempString.length() == 4) {
    userInputString = tempString; 
  }
  tempString = "";

  // Got new reading from IMU
  if (mpu.update()) {
    gyro_roll_input = mpu.getRoll();
    gyro_pitch_input = mpu.getPitch();
    gyro_yaw_input = mpu.getYaw();
  }

  // ===========================================
  // CHANGE SETPOINTS BASED ON BLUETOOTH SIGNAL
  // ===========================================

  /*
  // Choose setpoints based on data ("PRYT")
  pid_pitch_setpoint = userInputString.charAt(0) == '1' ? -SPEED : SPEED;
  if (userInputString.charAt(0) == '0') { pid_pitch_setpoint = 0; }

  pid_roll_setpoint = userInputString.charAt(1) == '1' ? -SPEED : SPEED;
  if (userInputString.charAt(1) == '0') { pid_roll_setpoint = 0; }

  pid_yaw_setpoint = userInputString.charAt(2) == '1' ? -SPEED : SPEED;
  if (userInputString.charAt(2) == '0') { pid_yaw_setpoint = 0; }
  */

  if (userInputString.charAt(3) != '0') {
    throttle += (userInputString.charAt(3) == '1' ? 1 : -1) * 2;
    if (throttle < 1100) { throttle = 1100; }
    if (throttle > 1800) { throttle = 1800; }
  }

  if (userInputString.charAt(0) != '0') {
    analogWrite(MOTOR_PIN_1, 0);
    analogWrite(MOTOR_PIN_2, 0);
    analogWrite(MOTOR_PIN_3, 0);
    analogWrite(MOTOR_PIN_4, 0);

    Serial.println("SAFETY SWITCH!!! SHUTTING OFF!!!");
    while (1) {}
  }

  Serial.println(throttle);
  // throttle = 1600;


  pid_pitch_setpoint = 0;
  pid_roll_setpoint = 0;
  pid_yaw_setpoint = 0;
  
  //Gyro angle calculations
  pitch_level_adjust = angle_pitch * 15;                                    //Calculate the pitch angle correction
  roll_level_adjust = angle_roll * 15;                                      //Calculate the roll angle correction

  calculate_pid();                                                            //PID inputs are known. So we can calculate the pid output.


  if (throttle > 1200){                                                     //The motors are started.
    if (throttle > 1800) throttle = 1800;                                   //We need some room to keep full control at full throttle.
    
    // esc_1 = throttle - pid_output_pitch; //Calculate the pulse for esc 1 (front-right - CCW)
    // esc_2 = throttle + pid_output_pitch; //Calculate the pulse for esc 2 (rear-right - CW)
    // esc_3 = throttle + pid_output_pitch; //Calculate the pulse for esc 3 (rear-left - CCW)
    // esc_4 = throttle - pid_output_pitch; //Calculate the pulse for esc 4 (front-left - CW)

    // esc_1 = throttle + pid_output_roll; //Calculate the pulse for esc 1 (front-right - CCW)
    // esc_2 = throttle + pid_output_roll; //Calculate the pulse for esc 2 (rear-right - CW)
    // esc_3 = throttle - pid_output_roll; //Calculate the pulse for esc 3 (rear-left - CCW)
    // esc_4 = throttle - pid_output_roll; //Calculate the pulse for esc 4 (front-left - CW)

    // esc_1 = throttle - pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW)
    // esc_2 = throttle + pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW)
    // esc_3 = throttle - pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
    // esc_4 = throttle + pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW)

    esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW)
    esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW)
    esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
    esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW)

    if (esc_1 < 1100) esc_1 = 1100;                                         //Keep the motors running.
    if (esc_2 < 1100) esc_2 = 1100;                                         //Keep the motors running.
    if (esc_3 < 1100) esc_3 = 1100;                                         //Keep the motors running.
    if (esc_4 < 1100) esc_4 = 1100;                                         //Keep the motors running.

    if(esc_1 > 2000)esc_1 = 2000;                                           //Limit the esc-1 pulse to 2000us.
    if(esc_2 > 2000)esc_2 = 2000;                                           //Limit the esc-2 pulse to 2000us.
    if(esc_3 > 2000)esc_3 = 2000;                                           //Limit the esc-3 pulse to 2000us.
    if(esc_4 > 2000)esc_4 = 2000;                                           //Limit the esc-4 pulse to 2000us.  
  }

  else{
    esc_1 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-1.
    esc_2 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-2.
    esc_3 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-3.
    esc_4 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-4.

    // Not armed
    angle_pitch = mpu.getPitch();                                          //Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
    angle_roll = mpu.getRoll();                                            //Set the gyro roll angle equal to the accelerometer roll angle when the quadcopter is started.

    //Reset the PID controllers for a bumpless start.
    pid_i_mem_roll = 0;
    pid_last_roll_d_error = 0;
    pid_i_mem_pitch = 0;
    pid_last_pitch_d_error = 0;
    pid_i_mem_yaw = 0;
    pid_last_yaw_d_error = 0;
  }

  // ===========================================
  // MAP OUTPUTS TO PWM SIGNALS
  // ===========================================

  // Write speeds
  analogWrite(MOTOR_PIN_1, esc_1 == 1000 ? 0 : map(esc_1, 1100, 1800, 60, 180));
  analogWrite(MOTOR_PIN_2, esc_2 == 1000 ? 0 : map(esc_2, 1100, 1800, 60, 180));
  analogWrite(MOTOR_PIN_3, esc_3 == 1000 ? 0 : map(esc_3, 1100, 1800, 0, 110));
  analogWrite(MOTOR_PIN_4, esc_4 == 1000 ? 0 : map(esc_4, 1100, 1800, 0, 110));

}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Subroutine for calculating pid outputs
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//The PID controllers are explained in part 5 of the YMFC-3D video session:
//https://youtu.be/JBvnB0279-Q 
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void calculate_pid() {
  //Roll calculations
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
  else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;

  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;

  pid_last_roll_d_error = pid_error_temp;

  //Pitch calculations
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;

  pid_last_pitch_d_error = pid_error_temp;

  //Yaw calculations
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;

  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;

  pid_last_yaw_d_error = pid_error_temp;
}


void print_roll_pitch_yaw() {
    Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(mpu.getYaw(), 2);
    Serial.print(", ");
    Serial.print(mpu.getPitch(), 2);
    Serial.print(", ");
    Serial.println(mpu.getRoll(), 2);
}

