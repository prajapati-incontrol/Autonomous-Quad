+////////////////////////////////////////
// The program will  start in calibration mode.
// Send the following characters / numbers via the serial monitor to change the mode.
//
//r = print receiver signals
//a = print quadcopter angles.
//1 = check rotation / vibrations for motor 1 (right front CCW).
//2 = check rotation / vibrations for motor 2 (right rear CW).
//3 = check rotation / vibrations for motor 3 (left rear CW).
//4 = check rotation / vibrations for motor 4 (left front CW).
//5 = check rotation / vibrations for all motors together.

#include<Wire.h>
#include<EEPROM.h>

//Declaring global variables
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
byte eeprom_data[36], start, data;
boolean new_function_request,first_angle;
volatile int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
int esc_1, esc_2, esc_3, esc_4;
int counter_channel_1, counter_channel_2, counter_channel_3, counter_channel_4;
int receiver_input[5];
int loop_counter, gyro_address, vibration_counter;
int temperature;
long acc_x, acc_y, acc_z, acc_total_vector[20], acc_av_vector, vibration_total_result;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long zero_timer, timer_1, timer_2, timer_3, timer_4, current_time;

int acc_axis[4], gyro_axis[4];
double gyro_pitch, gyro_roll, gyro_yaw;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
int cal_int;
double gyro_axis_cal[4];


void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  Wire.begin();
  TWBR = 12; // Setup I2C clock speed

  // Arduino Uno Pins default to inputs, so they don't need to be explicitly
  // DDRD maps pins D0 to D7. Here, D4, D5, D6 and D7 are the inputs to ESC signal. 
  DDRD |= B11110000; // pinMode(7,6,5,4, OUTPUT)
  DDRB |= B00010000; // pinMode(12,OUTPUT) for LED

  PCICR |= (1<<PCIE0); // set PCIE0 to enable PCMSK0 scan.
  PCMSK0 |= B00001111;

  // Read EEPROM for faster data access
  for(data = 0; data <= 35; data++)eeprom_data[data] = EEPROM.read(data);

  gyro_address = eeprom_data[32]; // recall the gyro address from YMFC setup

  set_gyro_registers(); // set the specific gyro registers
  // sets up the full scale range of gyro and accel; and 
  // sets a digital low pass filter of bandwidth 43kHz

  // Check the EEPROM sign
  //Check the EEPROM signature to make sure that the setup program is executed.
  while(eeprom_data[33] != 'J' || eeprom_data[34] != 'M' || eeprom_data[35] != 'B'){
    delay(500);                                                                         //Wait for 500ms.
    digitalWrite(12, !digitalRead(12));                                                 //Change the led status to indicate error.
  }

  wait_for_receiver(); // Wait until the receiver is active
  zero_timer = micros(); // set the zero_timer for the first loop

  while(Serial.available())data = Serial.read();  // Empty the Serial buffer.
  data = 0;  // Reset the zero timer. 
}

// Main program loop
void loop() {
  // put your main code here, to run repeatedly:
  while(zero_timer + 4000 > micros()); // Start the pulse after 4000 microseconds
  zero_timer = micros();
  
  //Every time the loop begins, it checks if any data is input from serial.
  //If no data is transmitted, then continue to the remaining part.
  if(Serial.available() > 0){
    data = Serial.read(); // Read the incoming byte
    delay(100);
    while(Serial.available() > 0)loop_counter = Serial.read(); // Empty Serial buffer 
    new_function_request = true; // set the new request flag
    loop_counter = 0;
    cal_int = 0;
    start = 0;
    first_angle = false;

    // Confirm the choice on serial monitor
    if(data == 'r')Serial.println("Reading receiver signals.");
    if(data == 'a')Serial.println("Print the quadcopter angles.");
    if(data == 'a')Serial.println("Gyro calibration starts in 2 seconds (don't move the quadcopter)");
    if(data == '1')Serial.println("Test motor 1 (right front CCW.)");
    if(data == '2')Serial.println("Test motor 2 (right rear CW.)");
    if(data == '3')Serial.println("Test motor 3 (left rear CCW.)");
    if(data == '4')Serial.println("Test motor 4 (left front CW.)");
    if(data == '5')Serial.println("Test all motors together");
    
    for(vibration_counter = 0; vibration_counter < 625; vibration_counter++){           //Do this loop 625 times
      delay(3);                                                                         //Wait 3000us.
      esc_1 = 1000;                                                                     //Set the pulse for ESC 1 to 1000us.
      esc_2 = 1000;                                                                     //Set the pulse for ESC 1 to 1000us.
      esc_3 = 1000;                                                                     //Set the pulse for ESC 1 to 1000us.
      esc_4 = 1000;                                                                     //Set the pulse for ESC 1 to 1000us.
      esc_pulse_output(); //Send the ESC control pulses.
      /*Creates the following type of square wave:
      ______                  _______
            |                |       |
            |                |       |
            |________________|       |______________
      HIGH is 1000us long and LOW is 3000us long
      So, for 4000us * 625 = 2.5s, the void loop will only display the message
      and send the above sq wave to ESCs. 
            */
    }
    vibration_counter = 0;
  }


  

  // after processing any data received through serial monitor, IF ANY.
  // begin with below...
  receiver_input_channel_3 = convert_receiver_channel(3);
  // So the program now checks if throttle stick is set to zero or 
  // close to zero so that calibration can begin.
  if (receiver_input_channel_3 < 1025) new_function_request = false;

  // Run the ESC calibration program to start with.
  // if no data is transmitted via the Serial Monitor and 
  // new function request = false
  if(data == 0 && new_function_request == false){
    receiver_input_channel_3 = convert_receiver_channel(3);
    esc_1 = receiver_input_channel_3;
    esc_2 = receiver_input_channel_3;
    esc_3 = receiver_input_channel_3;
    esc_4 = receiver_input_channel_3;
    esc_pulse_output(); // this function sends a pulse of width esc_1 to ESC-1
  }

  // When user sends a 'r' print the receiver signals
  if (data == 'r'){
    loop_counter ++;
    receiver_input_channel_1 = convert_receiver_channel(1);
    receiver_input_channel_2 = convert_receiver_channel(2);
    receiver_input_channel_3 = convert_receiver_channel(3);
    receiver_input_channel_4 = convert_receiver_channel(4);

    if (loop_counter == 125){
      print_signals();
      loop_counter = 0;
    }

    // For starting the motors: throttle low and yaw left (step 1).
    if(receiver_input_channel_3 < 1050 && receiver_input_channel_4 < 1050)start = 1;
    // When yaw stick is back in the center position start the motors (step 2).
    if(start == 1 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1450) start = 2;
    // stopping the motors: throttle low and yaw right
    if(start== 2 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1950)start = 0;

    esc_1 = 1000;
    esc_2 = 1000;
    esc_3 = 1000;
    esc_4 = 1000;
    esc_pulse_output();
  }

  // When user sends a '1', '2', '3', '4' or '5' to test the motors.
  if (data == '1' || data == '2' || data == '3' || data == '4' || data == '5'){
    loop_counter++;
    if(new_function_request == true && loop_counter == 250){
      
    }
  }

  
}

//This routine is called every time input 8, 9, 10 or 11 changed state.
ISR(PCINT0_vect){
  current_time = micros();
  //Channel 1=========================================
  if(PINB & B00000001){                                        //Is input 8 high?
    if(last_channel_1 == 0){                                   //Input 8 changed from 0 to 1.
      last_channel_1 = 1;                                      //Remember current input state.
      timer_1 = current_time;                                  //Set timer_1 to current_time.
    }
  }
  else if(last_channel_1 == 1){                                //Input 8 is not high and changed from 1 to 0.
    last_channel_1 = 0;                                        //Remember current input state.
    receiver_input[1] = current_time - timer_1;                 //Channel 1 is current_time - timer_1.
  }
  //Channel 2=========================================
  if(PINB & B00000010 ){                                       //Is input 9 high?
    if(last_channel_2 == 0){                                   //Input 9 changed from 0 to 1.
      last_channel_2 = 1;                                      //Remember current input state.
      timer_2 = current_time;                                  //Set timer_2 to current_time.
    }
  }
  else if(last_channel_2 == 1){                                //Input 9 is not high and changed from 1 to 0.
    last_channel_2 = 0;                                        //Remember current input state.
    receiver_input[2] = current_time - timer_2;                //Channel 2 is current_time - timer_2.
  }
  //Channel 3=========================================
  if(PINB & B00000100 ){                                       //Is input 10 high?
    if(last_channel_3 == 0){                                   //Input 10 changed from 0 to 1.
      last_channel_3 = 1;                                      //Remember current input state.
      timer_3 = current_time;                                  //Set timer_3 to current_time.
    }
  }
  else if(last_channel_3 == 1){                                //Input 10 is not high and changed from 1 to 0.
    last_channel_3 = 0;                                        //Remember current input state.
    receiver_input[3] = current_time - timer_3;                //Channel 3 is current_time - timer_3.
  }
  //Channel 4=========================================
  if(PINB & B00001000 ){                                       //Is input 11 high?
    if(last_channel_4 == 0){                                   //Input 11 changed from 0 to 1.
      last_channel_4 = 1;                                      //Remember current input state.
      timer_4 = current_time;                                  //Set timer_4 to current_time.
    }
  }
  else if(last_channel_4 == 1){                                //Input 11 is not high and changed from 1 to 0.
    last_channel_4 = 0;                                        //Remember current input state.
    receiver_input[4] = current_time - timer_4;                //Channel 4 is current_time - timer_4.
  }
}

void set_gyro_registers(){
  // Setup MPU-6050
  if(eeprom_data[31] == 1){ // if type == 1 which was MPU6050
    Wire.beginTransmission(gyro_address);
    //activating the gyro using PWR_MGMT register
    Wire.write(0x6B);
    Wire.write(0x00); 
    Wire.endTransmission();

    // Config full scale range of gyro using 1B register
    Wire.beginTransmission(gyro_address); 
    Wire.write(0x1B); // GYRO_CONFIG register
    Wire.write(0b00001000); // XG_ST YG_ST ZG_ST FS_SEL[1:0] - - - Full scale range of 500 deg/s
    Wire.endTransmission();

    // Config full scale range of accel using 1C register
    Wire.beginTransmission(gyro_address);
    Wire.write(0x1B); // ACCEL_CONFIG register
    Wire.write(0b00010000); // Full scale range of 8g
    Wire.endTransmission();

    // Setting the output bandwidth freq of 43Hz for both gyro and accel
    Wire.beginTransmission(gyro_address);
    Wire.write(0x1A); // We want to write to the CONFIG register 0x1A
    Wire.write(0b00000011); 
    Wire.endTransmission();    
  }
}

void wait_for_receiver(){
  byte zero = 0;
  while(zero < 15){
    if(receiver_input[1] < 2100 && receiver_input[1] > 900)zero |= 0b00000001;  //Set bit 0 if the receiver pulse 1 is within the 900 - 2100 range
    if(receiver_input[2] < 2100 && receiver_input[2] > 900)zero |= 0b00000010;  //Set bit 1 if the receiver pulse 2 is within the 900 - 2100 range
    if(receiver_input[3] < 2100 && receiver_input[3] > 900)zero |= 0b00000100;  //Set bit 2 if the receiver pulse 3 is within the 900 - 2100 range
    if(receiver_input[4] < 2100 && receiver_input[4] > 900)zero |= 0b00001000;  //Set bit 3 if the receiver pulse 4 is within the 900 - 2100 range
    delay(500);    
  }
}

void esc_pulse_output(){
  /*It creates the following square wave on all 4 dig pins:
  ________________
                  |
                  |
                  |
                  |
  <-  esc_x us  ->|______
        
        */
  zero_timer = micros();
  PORTD |= B11110000; // Set D7, D6, D5, D4 to high
  //Calculate the time when digital port 4 is set LOW
  // which is equal to the time it takes to make the port LOW 
  // plus the time since the code has begun.
  timer_channel_1 = esc_1 + zero_timer; 
  //Calculate the time when digital port 5 is set LOW
  timer_channel_2 = esc_2 + zero_timer;
  //Caculate the tiem when digital port 6 is set LOW
  timer_channel_3 = esc_3 + zero_timer;
  //Caculate the time when digital port 7 is set LOW
  timer_channel_4 = esc_4 + zero_timer;

  // 16 = B00010000
  // D4-7High = B11110000
  // Until D4 or D5 or D6 or D7 are high, keep runnings.
  // else get out.
  // Execute the loop until digital ports 4 to 6 are low.
  while(PORTD >= 16){
    esc_loop_timer = micros(); // chk the current time
    // When the delay time is expired, digital port 4 is set LOW
    // Delay time here is the time it took to go from 
    if(timer_channel_1 <= esc_loop_timer)PORTD &= B11101111;
    if(timer_channel_2 <= esc_loop_timer)PORTD &= B11011111;
    if(timer_channel_3 <= esc_loop_timer)PORTD &= B10111111;
    if(timer_channel_4 <= esc_loop_timer)PORTD &= B01111111;
  }
}
  
int convert_receiver_channel(byte function){
  byte channel, reverse;
  int low, center, high, actual;
  int difference;
  
  channel = eeprom_data[function + 23] & 0b00000111;
  // first call: channel = B00001010 & B00000111;
  // which is = B00000010 = 2 meaning throttle
  // channel = 0 meaning roll, 
  // channel 1 = meaning pitch
  // channel 3 meaning yaw
  if(eeprom_data[function+23] & 0b10000000)reverse = 1;
  else reverse = 0;
  
  actual = receiver_input[channel];
  // retreiving the low of the corresponding channel
  low = (eeprom_data[channel*2 + 15] << 8) | eeprom_data[channel*2 + 14];
  center = (eeprom_data[channel*2 -1]<< 8) | eeprom_data[channel*2 - 2];
  high = (eeprom_data[channel*2 + 7] << 8) | eeprom_data[channel*2 + 6];

  // lets say actual = 1212; low = 1004; center = 1504
  // so, to map the actual to low = 1000 & high = 2000
  // refer mackinac bridgeY
  if (actual < center){
    if(actual < low) actual = low;
    // calculate and scale the value  to 1000 - 2000us
    difference = ((long)(center - actual) * (long)500) / (center - low);
    if(reverse == 1) return 1500 + difference
    else return 1500 - difference
  }
  else if (actual > center){
    if (actual > high) actual = high;
    difference = ((long)(actual - center) * (long)500) / (high - center);
    if (reverse == 1) return 1500 - difference
    else return 1500 + difference;
  }
  else return 1500;
}

void print_signals(){
  Serial.print("Start: ");
  Serial.print(start); // 0

  Serial.print("  Roll:");
  if(receiver_input_channel_1 - 1480 < 0)Serial.print("<<<");
  else if(receiver_input_channel_1 - 1520 > 0)Serial.print(">>>");
  else Serial.print("-+-");
  Serial.print(receiver_input_channel_1);

  Serial.print("  Pitch:");
  if(receiver_input_channel_2 - 1480 < 0)Serial.print("^^^");
  else if(receiver_input_channel_2 - 1520 > 0)Serial.print("vvv");
  else Serial.print("-+-");
  Serial.print(receiver_input_channel_2);

  Serial.print("  Throttle:");
  if(receiver_input_channel_3 - 1480 < 0)Serial.print("vvv");
  else if(receiver_input_channel_3 - 1520 > 0)Serial.print("^^^");
  else Serial.print("-+-");
  Serial.print(receiver_input_channel_3);

  Serial.print("  Yaw:");
  if(receiver_input_channel_4 - 1480 < 0)Serial.print("<<<");
  else if(receiver_input_channel_4 - 1520 > 0)Serial.print(">>>");
  else Serial.print("-+-");
  Serial.println(receiver_input_channel_4);
  
}
