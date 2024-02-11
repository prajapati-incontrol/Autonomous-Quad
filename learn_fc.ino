#include<Wire.h>
#include<EEPROM.h>

float kproll = 1.3;
float kiroll = 0.04;
float kdroll = 18.0;
int pid_max_roll = 400; // Maximum output of the PID-controller for roll (+/-)

float kppitch = kproll;
float kipitch = kiroll;
float kdpitch = kdroll;
int pid_max_pitch = pid_max_roll;

float kpyaw = 4.0;
float kiyaw = 0.02;
float kdyaw = 0.0;
int pid_max_yaw = 400;

boolean auto_level = true; // Auto level on (true) or off (false)

//Declaring global variables
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
byte eeprom_data[36];
byte highByte, lowByte;
volatile int receiver_input_channel_1, receiver_input_channel_2, \
receiver_input_channel_3, receiver_input_channel_4;
int counter_channel_1, counter_channel_2, counter_channel_3, counter_channel_4;
int esc_1, esc_2, esc_3, esc_4;
int throttle, battery_voltage;
int cal_int, start, gyro_address;
int receiver_input[5];
int temperature;
int acc_axis[4], gyro_axis[4];
float roll_level_adjust, pitch_level_adjust;

long acc_x, acc_y, acc_z, acc_total_vector;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
unsigned long loop_timer;
double gyro_pitch, gyro_roll, gyro_yaw;
double gyro_axis_cal[4];
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
boolean gyro_angles_set;

// Setup routine
void setup() {
  //Serial.begin(57600)
  //Copy the EEPROM data for fast access data.
  for (start = 0; start <= 35; start++)eeprom_data[start] == EEPROM.read(start);
  start = 0;
  gyro_address = eeprom_data[32];

  Wire.begin();

  // Setting up the clock speed
  TWBR = 12;

  //Arduino (Atmega) pins default to inputs, so they don't need to be explicitly declared as inputs.
  DDRD |= B11110000;
  DDRB |= B00110000; // pinMode(13 & 12,OUTPUT)

  //Use the led on the arduino for startup indication.
  digitalWrite(12, HIGH);

  //Check the EEPROM signature to make sure that the setup program is execulted
  while (eeprom_data[33] != 'J' || eeprom_data[34] != 'M' || eeprom_data[35] != 'B')delay(10);

  //The flight controller needs the MPU6050 with gyro and accelerometer
  //If the setup is completed without MPU6050 stop the flight controller program
  if (eeprom_data[31] == 2 || eeprom_data[31] == 3)delay(10);

  set_gyro_registers();

  for (cal_int = 0; cal_int < 1250 ; cal_int ++) {                          //Wait 5 seconds before continuing.
    PORTD |= B11110000;                                                     //Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                                                //Wait 1000us.
    PORTD &= B00001111;                                                     //Set digital poort 4, 5, 6 and 7 low.
    delayMicroseconds(3000);                                                //Wait 3000us.
  }
  //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
  for (cal_int = 0; cal_int < 2000 ; cal_int ++) {                          //Take 2000 readings for calibration.
    if (cal_int % 15 == 0)digitalWrite(12, !digitalRead(12));               //Change the led status to indicate calibration.
    gyro_signalen();                                                        //Read the gyro output.
    gyro_axis_cal[1] += gyro_axis[1];                                       //Ad roll value to gyro_roll_cal.
    gyro_axis_cal[2] += gyro_axis[2];                                       //Ad pitch value to gyro_pitch_cal.
    gyro_axis_cal[3] += gyro_axis[3];                                       //Ad yaw value to gyro_yaw_cal.
    //We don't want the esc's to be beeping annoyingly. So let's give them a 1000us puls while calibrating the gyro.
    PORTD |= B11110000;                                                     //Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                                                //Wait 1000us.
    PORTD &= B00001111;                                                     //Set digital poort 4, 5, 6 and 7 low.
    delay(3);                                                               //Wait 3 milliseconds before the next loop.
  }

  //Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset.
  gyro_axis_cal[1] /= 2000;                                                 //Divide the roll total by 2000.
  gyro_axis_cal[2] /= 2000;                                                 //Divide the pitch total by 2000.
  gyro_axis_cal[3] /= 2000;                                                 //Divide the yaw total by

  PCICR |= (1 << PCIE0);                                                    //Set PCIE0 to enable PCMSK0 scan.
  PCMSK0 |= (1 << PCINT0);                                                  //Set PCINT0 (digital input 8) to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT1);                                                  //Set PCINT1 (digital input 9)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT2);                                                  //Set PCINT2 (digital input 10)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT3);                                                  //Set PCINT3 (digital input 11)to trigger an interrupt on state change.

  //Wait until the receiver is active and the throttle is set to the lower position.
  while (receiver_input_channel_3 < 990 || receiver_input_channel_3 > 1020 || receiver_input_channel_4 < 1400) {
    receiver_input_channel_3 = convert_receiver_channel(3);
    receiver_input_channel_4 = convert_receiver_channel(4);
    start++;
    PORTD |= B11110000;
    delayMicroseconds(1000);
    PORTD &= B00001111;
    delay(3);
    if (start == 125) {
      digitalWrite(12, !digitalRead(12));
      start = 0;
    }
  }
  start = 0;

  //Load the battery voltage to the battery voltage variable
  // 65 is the voltage compensation for the diode.
  // 12.6V eqals ~5V @ analog 0.
  // 12.6V equals 1023 analogRead(0)
  // 1260 / 1023 = 1.2317
  // The variable battery_voltage holds 1050 if the battery voltage is 10.5V
  battery_voltage = (analogRead(0) + 65) * 1.2317;

  loop_timer = micros();

  //When everythin is done, turn off the led.
  digitalWrite(12, LOW);
}

//MAIN PROGRAM LOOP

void loop() {

  // Complementary filter is implemented.
  //65.5 = 1 deg/sec (check the datasheet of the MPU-6050 for more information).
  gyro_roll_input = (gyro_roll_input * 0.7) + ((gyro_roll / 65.5) * 0.3);   //Gyro pid input is deg/sec.
  gyro_pitch_input = (gyro_pitch_input * 0.7) + ((gyro_pitch / 65.5) * 0.3); //Gyro pid input is deg/sec.
  gyro_yaw_input = (gyro_yaw_input * 0.7) + ((gyro_yaw / 65.5) * 0.3);      //Gyro pid input is deg/sec.

  // integrating the gyro raw values to angles in degrees
  angle_pitch += gyro_pitch * 0.0000611;
  angle_roll += gyro_roll * 0.0000611;

  //adding coupling to the calculation
  angle_pitch -= angle_roll * sin(gyro_yaw * 0.000001066);
  angle_roll += anlge_pitch * sin(gyro_yaw * 0.000001066);

  //accelerometer angle calculations
  acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));

  // calculating the pitch angle using acc values
  if (abs(acc_y) < acc_total_vector) { // prevent asin to produce nan
    // asin func returns value in radians, so converting to deg * by 57.296
    angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296;
  }

  // calculating the roll angle using acc values
  if (abs(acc_x) < acc_total_vector) {
    angle_roll_acc = asin((float)acc_x / acc_total_vector) * -57.296;
  }

  // With the help of a spirit level, first check if the gyro is perfectly horizontal
  // and note down the acc values. Use these values to calibrate the accelerometer
  // here we use 0.0
  angle_pitch_acc -= 0.0;
  angle_roll_acc -= 0.0;

  // Complementary filter
  angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;
  angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;

  //
  pitch_level_adjust = angle_pitch * 15;
  roll_level_adjust = angle_roll * 15;

  // auto_level = true in global definition
  // not coding for autolevel then adjust = 0;
  if (!auto_level) {
    pitch_level_adjust = 0;
    roll_level_adjust = 0;
  }

  //For starting the motors: throttle low and yaw left (step 1).
  if (receiver_input_channel_3 < 1050 && receiver_input_channel_4 < 1050)start = 1;
  //When yaw stick is back in the center position start the motors (step 2).
  if (start == 1 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1450) {
    start = 2;

    // For when the quadcopter is code is started, consider pitch and
    // roll angles to be of that of accelerometer
    angle_pitch = angle_pitch_acc;
    angle_roll = angle_roll_acc;
    gyro_angles_set = true;

    // Reset the PID controllers for bumpless start.
    pid_i_mem_roll = 0;
    pid_last_roll_d_error = 0;
    pid_i_mem_pitch = 0;
    pid_last_pitch_d_error = 0;
    pid_i_mem_yaw = 0;
    pid_last_yaw_d_error = 0;
  }
  // Stopping the motors: throttle low and yaw right.
  if(start == 2 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1950)start = 0;

  //The PID set point in deg/s is determined by the roll receiver input. 
  // In the case of dividing by 3 the max roll rate is approx 164 deg/s (500/3)
  pid_roll_setpoint = 0;
  // We need a little dead band of 16us for better results.
  if(receiver_input_channel_1 > 1508)pid_roll_setpoint = receiver_input_channel_1 - 1508;
  else if(receiver_input_channel_1 < 1492)pid_roll_setpoint = receiver_input_channel_1 

  pid_roll_setpoint -= roll_level_adjust;
  pid_roll_setpoint /= 3.0;

  //The PID setpoint in deg/s is determined by the pitch receiver input.
  //In the case of dividing by 3 the max pitch rate is aprox 164 deg/s
  pid_pitch_setpoint = 0;
  //We need a little deadband of 16us for better results
  if(receiver_input_channel_2 > 1508)pid_pitch_setpoint = receiver_input_channel_2 - 1508;
  else if(receiver_input_channel_2 < 1492)pid_pitch_setpoint = receiver_input_channel_2 - 1492;

  pid_pitch_setpoint -= pitch_level_adjust;
  pid_pitch_setpoint /= 3.0;

  //The PID setpoint in deg/s is determined by the yaw receiver input.
  //In the case of dividing by 3 the max yaw rate is aprox 164 deg/s
  pid_yaw_setpoint = 0;
  //We need a little dead band of 16us for better results.
  //Do not yaw when turning off the motors 
  if(receiver_input_channel_3 > 1050){
    if(receiver_input_channel_3 > 1050)pid_yaw_setpoint = (receiver_input_channel_4 - 1508)/3.0;
    else if(receiver_input_channel_4 < 1492)pid_yaw_setpoint = (receiver_input_channel_4 - 1492)/3.0;
  }

  calculate_pid();

  //The battery voltage is needed for compensation
  //A complementary filter is used to reduce noise.
  // 0.09853 = 0.08 * 1.2317
  battery_voltage = battery_voltage * 0.92 + (analogRead(0) + 65) * 0.09853;

  // Turn on the led if the battery voltage is to low.
  if(battery_voltage < 1000 && battery_voltage > 600)digitalWrite(12,HIGH);

  // INPUTS to the ESCs as per the motor mixing algorithm
  throttle = receiver_input_channel_3;

  if(start == 2){
    if(throttle > 1800) throttle = 1800;
    esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw;
    esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw;
    esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw;
    esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw;

    // do learn the working of pwm signal with battery to understand the 
    // below code. the below code compensates te voltage drop in order to 
    // keep the rpm constant. when voltage drops lets say 1100, then the esc which 
    // was 1600 will be now 1709.14. The below model looks an empirical one. 
    if(battery_voltage < 1240 && battery_voltage > 800){
      esc_1 += esc_1 * ((1240 - battery_voltage)/(float)3500);
      esc_2 += esc_2 * ((1240 - battery_voltage)/(float)3500);
      esc_3 += esc_3 * ((1240 - battery_voltage)/(float)3500);
      esc_4 += esc_4 * ((1240 - battery_voltage)/(float)3500);
    }

    if (esc_1 < 1100) esc_1 = 1100;
    if (esc_2 < 1100) esc_2 = 1100;
    if (esc_3 < 1100) esc_3 = 1100;
    if (esc_4 < 1100) esc_4 = 1100;

    if(esc_1 > 2000)esc_1 = 2000;                                           //Limit the esc-1 pulse to 2000us.
    if(esc_2 > 2000)esc_2 = 2000;                                           //Limit the esc-2 pulse to 2000us.
    if(esc_3 > 2000)esc_3 = 2000;                                           //Limit the esc-3 pulse to 2000us.
    if(esc_4 > 2000)esc_4 = 2000;  
  }

  else {
    esc_1 = 1000;
    esc_2 = 1000;
    esc_3 = 1000;
    esc_4 = 1000;
  }

  // ! ! ! ! ! ! !
  //Because of the angle calculation the loop time is getting very important.
  // longer or shorter than 4000us the angle calculation is off. if you modify the code 
  // make sure the loop time is still 4000us and no logner! 

  if (micros() - loop_timer > 4050)digitalWrite(12,HIGH);

  while(micros() - loop_timer < 4000);
  loop_timer = micros();

  PORTD |= B11110000;
  timer_channel_1 = esc_1 + loop_timer;
  timer_channel_2 = esc_2 + loop_timer;
  timer_channel_3 = esc_3 + loop_timer;
  timer_channel_4 = esc_4 + loop_timer;
  
}

ISR(PCINT0_vect) {
  current_time = micros();
  //Channel 1=========================================
  if (PINB & B00000001) {                                                   //Is input 8 high?
    if (last_channel_1 == 0) {                                              //Input 8 changed from 0 to 1.
      last_channel_1 = 1;                                                   //Remember current input state.
      timer_1 = current_time;                                               //Set timer_1 to current_time.
    }
  }
  else if (last_channel_1 == 1) {                                           //Input 8 is not high and changed from 1 to 0.
    last_channel_1 = 0;                                                     //Remember current input state.
    receiver_input[1] = current_time - timer_1;                             //Channel 1 is current_time - timer_1.
  }
  //Channel 2=========================================
  if (PINB & B00000010 ) {                                                  //Is input 9 high?
    if (last_channel_2 == 0) {                                              //Input 9 changed from 0 to 1.
      last_channel_2 = 1;                                                   //Remember current input state.
      timer_2 = current_time;                                               //Set timer_2 to current_time.
    }
  }
  else if (last_channel_2 == 1) {                                           //Input 9 is not high and changed from 1 to 0.
    last_channel_2 = 0;                                                     //Remember current input state.
    receiver_input[2] = current_time - timer_2;                             //Channel 2 is current_time - timer_2.
  }
  //Channel 3=========================================
  if (PINB & B00000100 ) {                                                  //Is input 10 high?
    if (last_channel_3 == 0) {                                              //Input 10 changed from 0 to 1.
      last_channel_3 = 1;                                                   //Remember current input state.
      timer_3 = current_time;                                               //Set timer_3 to current_time.
    }
  }
  else if (last_channel_3 == 1) {                                           //Input 10 is not high and changed from 1 to 0.
    last_channel_3 = 0;                                                     //Remember current input state.
    receiver_input[3] = current_time - timer_3;                             //Channel 3 is current_time - timer_3.

  }
  //Channel 4=========================================
  if (PINB & B00001000 ) {                                                  //Is input 11 high?
    if (last_channel_4 == 0) {                                              //Input 11 changed from 0 to 1.
      last_channel_4 = 1;                                                   //Remember current input state.
      timer_4 = current_time;                                               //Set timer_4 to current_time.
    }
  }
  else if (last_channel_4 == 1) {                                           //Input 11 is not high and changed from 1 to 0.
    last_channel_4 = 0;                                                     //Remember current input state.
    receiver_input[4] = current_time - timer_4;                             //Channel 4 is current_time - timer_4.
  }
}

void set_gyro_registers() {
  if (eeprom_data[31] == 1) {
    Wire.beginTransmission(gyro_address);
    // power management register
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();

    Wire.beginTransmission(gyro_address);
    // gyro config for full scale range setup = +/- 500 deg/s
    Wire.write(0x1B);
    Wire.write(0x08);
    Wire.endTransmission();

    Wire.beginTransmission(gyro_address);
    // accel config for full scale range of +/- 4g
    Wire.write(0x1C);
    Wire.write(0x10);
    Wire.endTransmission();

    //Let's perform a random register check to see if the values are written correctly
    Wire.beginTransmission(gyro_address);
    Wire.write(0x1B);
    Wire.endTransmission();
    Wire.requestFrom(gyro_address, 1);
    while (Wire.available() < 1);
    if (Wire.read() != 0x08) {
      digitalWrite(12, HIGH);
      while (1)delay(10);
    }

    Wire.beginTransmission(gyro_address);
    // for FSYNC and DLPF
    Wire.write(0x1A);
    Wire.write(0x03);
    Wire.endTransmission();
  }
}

void gyro_signalen() {
  if (eeprom_data[31] == 1) {
    Wire.beginTransmission(gyro_address);
    Wire.write(0x3B);
    Wire.endTransmission();
    Wire.requestFrom(gyro_address, 14);

    receiver_input_channel_1 = convert_receiver_channel(1);
    receiver_input_channel_2 = convert_receiver_channel(2);
    receiver_input_channel_3 = convert_receiver_channel(3);
    receiver_input_channel_4 = convert_receiver_channel(4);

    while (Wire.available() < 14);
    acc_axis[1] = Wire.read() << 8 | Wire.read();
    acc_axis[2] = Wire.read() << 8 | Wire.read();
    acc_axis[3] = Wire.read() << 8 | Wire.read();
    temperature = Wire.read() << 8 | Wire.read();                           //Add the low and high byte to the temperature variable.
    gyro_axis[1] = Wire.read() << 8 | Wire.read();                          //Read high and low part of the angular data.
    gyro_axis[2] = Wire.read() << 8 | Wire.read();                          //Read high and low part of the angular data.
    gyro_axis[3] = Wire.read() << 8 | Wire.read();
  }

  if (cal_int == 2000) {
    gyro_axis[1] -= gyro_axis_cal[1];
    gyro_axis[2] -= gyro_axis_cal[2];
    gyro_axis[3] -= gyro_axis_cal[3];
  }

  // gyro roll = gyro_axis[1]
  gyro_roll = gyro_axis[eeprom_data[28] & 0b00000011];
  // below statement checks if the axis is inverted.
  if (eeprom_data[28] & 0b10000000)gyro_roll *= -1;
  gyro_pitch = gyro_axis[eeprom_data[29] & 0b00000011];
  if (eeprom_data[29] & 0b10000000)gyro_pitch *= -1;
  gyro_yaw = gyro_axis[eeprom_data[30] & 0b00000011];
  if (eeprom_data[30] & 0b10000000)gyro_yaw *= -1;


  // set the correct axis for acc.
  acc_x = acc_axis[eeprom_data[29] & 0b00000011];                           //Set acc_x to the correct axis that was stored in the EEPROM.
  if (eeprom_data[29] & 0b10000000)acc_x *= -1;                             //Invert acc_x if the MSB of EEPROM bit 29 is set.
  acc_y = acc_axis[eeprom_data[28] & 0b00000011];                           //Set acc_y to the correct axis that was stored in the EEPROM.
  if (eeprom_data[28] & 0b10000000)acc_y *= -1;                             //Invert acc_y if the MSB of EEPROM bit 28 is set.
  acc_z = acc_axis[eeprom_data[30] & 0b00000011];                           //Set acc_z to the correct axis that was stored in the EEPROM.
  if (eeprom_data[30] & 0b10000000)acc_z *= -1;
}

void calculate_pid(){
  //Roll calculations
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
  else if (pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;
  
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
