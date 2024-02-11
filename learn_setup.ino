// This code mainly uses the codes of JOOP BROKKING from Youtube. Only comments and slight modifications 
// are made by me.

// Setup Routine
void setup(){
  pinMode(12, OUTPUT);
  /* Arduino (Atmega) pins default to inputs, so they don't need to 
  be explicitly declared as inputs*/
  PCICR = 0b00000001; // set PCIE0 to enable PSMSK0 scan which scans pins D8-D13 for 
                      // the Pin Change Interrupt
  PCMSK0 = 0b00001111; // set PCINT0 - PCINT3 or D8-D11 to trigger an interrupt on state change
  
  Wire.begin();       // start the I2C master
  Serial.begin(57600);     // start the serial connection at 57,600 buad per second
  delay(250);         // 250 micro seconds delay before void loop runs.    
}

// Main program
void loop(){
  // Show the YMFC-3D V2 intro
  intro(); // just some comments introducing the project and the author

  Serial.println(F(""));
  Serial.println(F("====================================="));
  Serial.println(F("System check"));
  delay(1000);
  Serial.println(F("Checking I2C clock speed."));
  delay(1000);

  TWBR = 12;            // Set the clock speed to 400kHz

  #if F_CPU == 16000000L // 16MHz
    clockspeed_ok = 1;
  #endif

  // 1st check: Check the clockspeed
  if(TWBR == 12 && clockspeed_ok){
    Serial.println(F("I2C clock speed is correctly set to 400kHz"));
  } else {
    Serial.println(F("I2C clock speed is not set to 400kHz. (ERROR 8)"));
    error = 1;
  }

  // 2nd check: Check whether the receiver signals are valid, lie within [900,2100]
  if (error == 0){
    Serial.println(F(""));
    Serial.println(F("==================================================="));
    Serial.println(F("Transmitter setup"));
    Serial.println(F("==================================================="));
    delay(1000);
    Serial.print(F("Checking for valid receiver signals."));
    // wait 10 seconds until all receiver inputs are valid
    wait_for_receiver();
    Serial.println(F(""));
  }
  
  // Quit the program in case of an error
  // 3rd check: Store the center positions of all the sticks.
  if(error == 0){
    delay(2000);
    Serial.println(F("Place all sticks and subtrims in the center position within 10 seconds"));
    // Showing the timer on the Serial Monitor
    for (int i = 9; i > 0; i--){
      delay(1000);
      Serial.print(i);
      Serial.print(" ");
    }
    Serial.print(" ");
    //Store the central stick positions
    center_channel_1 = receiver_input_channel_1;
    center_channel_2 = receiver_input_channel_2;
    center_channel_3 = receiver_input_channel_3;
    center_channel_4 = receiver_input_channel_4;
    Serial.print(F(""));
    Serial.println(F("Center positions stored."));
    Serial.print(F("Digital Input 08 = "));
    Serial.println(receiver_input_channel_1);
    Serial.print(F("Digital Input 09 = "));
    Serial.println(receiver_input_channel_2);
    Serial.print(F("Digital Input 10 = "));
    Serial.println(receiver_input_channel_3);
    Serial.print(F("Digital Input 11 = "));
    Serial.println(receiver_input_channel_4);
    Serial.println(F(""));
    Serial.println(F(""));
  }
  
  // 4th check
  if (error == 0){
    Serial.println(F("Move the throttle stick to full throttle and back to center"));
    // check for throttle movement
    check_receiver_inputs(1); // channel_3_assign
    Serial.print(F("Throttle is connected to digital input "));
    Serial.println((channel_3_assign & 0b00000111) + 7);
    // channel_3_assign = 3 => 0b00000011 & 0b00000111 => 3 + 7 = 10 = 0b000001000;
    // Throttle is connected to digital input 10.
    if(channel_3_assign & 0b10000000) Serial.println(F("Channel inverted = yes"));
    else Serial.println(F("Channel inverted = no"));
    wait_sticks_zero();

    // 5th check
    Serial.println(F(""));
    Serial.println(F(""));
    Serial.println(F("Move the roll stick to simulate left wing up and back to center"));
    // Check for roll movement
    check_receiver_inputs(2);
    Serial.print(F("Roll is connected to digital input "));
    Serial.println((channel_1_assign & 0b00000111) + 7);
    // channel_1_assign = 1 or 0b00000001 => 1 + 7 = 8 or 0b00001000;
    // Roll is connected to digital input 8
    if (channel_1_assign & 0b10000000)Serial.println(F("Right wing up condition: Channel inverted = yes"));
    else Serial.println(F("Channel inverted = no"));
    wait_sticks_zero();
  }
   if(error == 0){
    Serial.println(F(""));
    Serial.println(F(""));
    Serial.println(F("Move the pitch stick to simulate nose up and back to center"));
    //Check for throttle movement
    check_receiver_inputs(3);
    Serial.print(F("Pitch is connected to digital input "));
    Serial.println((channel_2_assign & 0b00000111) + 7);
    // channel_2_assign = 2; 0b00000010 & 0b00000111 => 2 + 7 = 9
    // Pitch is connected to digital input 9
    if(channel_2_assign & 0b10000000)Serial.println(F("Nose down condition: Channel inverted = yes"));
    else Serial.println(F("Channel inverted = no"));
    wait_sticks_zero();
  }
   if(error == 0){
    Serial.println(F(""));
    Serial.println(F(""));
    Serial.println(F("Move the yaw stick to simulate nose right and back to center"));
    //Check for throttle movement
    check_receiver_inputs(4);
    Serial.print(F("Yaw is connected to digital input "));
    Serial.println((channel_4_assign & 0b00000111) + 7);
    // Yaw is connected to digital input 11
    if(channel_4_assign & 0b10000000)Serial.println(F("Nose turns left condition: Channel inverted = yes"));
    else Serial.println(F("Channel inverted = no"));
    wait_sticks_zero();
  }

  // 8th check.
  if (error == 0){
    Serial.println(F(""));
    Serial.println(F(""));
    Serial.println(F("Gently move all the sticks simultaneosly to their extends"));
    // Register the min and max values of the receiver channels
    register_min_max();
    Serial.println(F(""));
    Serial.println(F(""));
    Serial.println(F("High, low and center values found during setup"));
    Serial.print(F("Digital input 08 (Roll) values: "));
    Serial.print(low_channel_1);
    Serial.print(F(" - "));
    Serial.print(center_channel_1);
    Serial.print(F(" - "));
    Serial.print(high_channel_1);
    
    Serial.print(F("Digital input 09 (Pitch) values: "));
    Serial.print(low_channel_2);
    Serial.print(F(" - "));
    Serial.print(center_channel_2);
    Serial.print(F(" - "));
    Serial.print(high_channel_2);

    Serial.print(F("Digital input 10 (Throttle) values: "));
    Serial.print(low_channel_3);
    Serial.print(F(" - "));
    Serial.print(center_channel_3);
    Serial.print(F(" - "));
    Serial.print(high_channel_3);

    Serial.print(F("Digital input 11 (Yaw) values: "));
    Serial.print(low_channel_4);
    Serial.print(F(" - "));
    Serial.print(center_channel_4);
    Serial.print(F(" - "));
    Serial.print(high_channel_4);

    Serial.println(F("Move stick 'nose up' and back to center to continue"));
    check_to_continue();
  }

  // 9th Check: Check the options for GYROs
  if (error == 0){
    // What gyro is connected? 
    // Options: MPU-6050; L3G4200D; L3GD20H; 
    Serial.println(F(""));
    Serial.println(F("=============================="));
    Serial.println(F("Gyro Search"));
    Serial.println(F("=============================="));
    delay(2000);

    Serial.println(F("Searching for MPU-6050 on address 0x68/104"));
    delay(1000);
    if (search_gyro(0x68, 0x75) == 0x68){
      Serial.println(F("MPU-6050 found on address 0x68!"));
      type = 1;
      gyro_address = 0x68;
    }

    if (type == 0){
      Serial.println(F("Searching for MPU6050 on address 0x68/105"));
      delay(1000);
      if (search_gyro(0x69, 0x75) == 0x68){
        Serial.println(F("MPU6050 found on address 0x69"));
        type = 1;
        gyro_address = 0x69;
      }
    }

    if (type == 0){
      Serial.println(F("Searching L3G4200D on address 0x68/104"));
      type = 2;
      if (search_gyro(0x68, 0x0F) == 0xD3){
        Serial.println(F("L3G4200D found on address 0x68"));
        type = 2;
        gyro_address = 0x68;
      }
    }

    if (type == 0){
      Serial.println(F("Searching for L34200D on address 0x69/105"));
      delay(1000);
      if (search_gyro(0x69, 0x0F) == 0xD3){
        Serial.println(F("L3G4200D found on address 0x69"));
        type = 2;
        gyro_address = 0x69;
      }
    }
    
    if (type == 0){
      Serial.println(F("Searching for L3GD20H on address 0x6A/106"));
      delay(1000);
      if (search_gyro(0x6A, 0X0F) == 0XD7){
        Serial.println(F("L3GD20H found on address 0x6A"));
        type = 3;
        gyro_address = 0x6A;
      }
    }
    
    if(type == 0){
     Serial.println(F("Searching for L3GD20H on address 0x6B/107"));
      delay(1000);
      if(search_gyro(0x6B, 0x0F) == 0xD7){
        Serial.println(F("L3GD20H found on address 0x6B"));
        type = 3;
        gyro_address = 0x6B;
      }
    }

    if(type == 0){
      Serial.println(F("No gyro device found!!! (ERROR 3)"));
      error = 1;
    }

    // if the gyro is found we can setup the correct gyro axes.
    else{
      delay(3000);
      Serial.println(F(""));
      Serial.println(F("=================================="));
      Serial.println(F("Gyro register settings"));
      Serial.println(F("=================================="));
      start_gyro(); // setup the gyro for further use
    }
  }

  // 11th Check: if the gyro is found we can setup the correct gyro axes.
  if (error == 0){
    delay(3000);
    Serial.println(F(""));
    Serial.println(F("===================================="));
    Serial.println(F("Gyro Calibration"));
    Serial.println(F("===================================="));
    Serial.println(F("Don't move the quadcopter!! Calibration starts in 3 seconds"));
    delay(3000);
    Serial.println(F("Calibrating the gyro, this will take +/- 8 seconds"));
    Serial.print(F("Please wait"));
    // Let's take the multiple gyro data samples so we can determine the average gyro offset (calibration)
    for (cal_int = 0; cal_int < 2000; can_int++){           // Take 2000 readings for calibration
      if (cal_int % 100 == 0)Serial.print(F("."));          // Print dot to indicate calibraion
      gyro_signalen();                                      // Read the gyro output.
      gyro_roll_cal += gyro_roll;
      gyro_pitch_cal += gyro_pitch;
      gyro_yaw_cal += gyro_yaw;
      delay(4);
    }
    // Now that we have 2000 measures, we need to divie by 2000 to get average.
    gyro_roll_cal /= 2000;
    gyro_pitch_cal /= 2000;
    gyro_yaw_cal /= 2000;

    // Show the calibration results
    Serial.println(F(""));
    Serial.print(F("Axis 1 offset="));
    Serial.println(gyro_roll_cal);
    Serial.print(F("Axis 2 offset="));
    Serial.println(gyro_pitch_cal);
    Serial.print(F("Axis 3 offset="));
    Serial.println(gyro_yaw_cal);
    Serial.println(F(""));
    
    Serial.println(F("==================================================="));
    Serial.println(F("Gyro axes configuration"));
    Serial.println(F("==================================================="));

    // Detect the lift wing up movement 
    Serial.println(F("Lift the left side of the quadcopter to a 45 degree angle within 10 seconds"));
    //Check axis movement
    check_gyro_axes(1);
    if (error == 0){
      Serial.println(F("OK!"));
      Serial.print(F("Angle detection = "));
      Serial.println(roll_axis & 0b00000011); // when roll_axis = 1 which is +ve roll
      if (roll_axis & 0b10000000)Serial.println(F("Axis inverted = yes")); // when roll_axis = 129
      else Serial.println(F("Axis inverted = no"));
      Serial.println(F("Put the quadcopter back in its original position"));
      Serial.println(F("Move stick 'nose up' and back to center to continue"));
      check_to_continue();

      //Detect the nose up movement
      Serial.println(F(""));
      Serial.println(F(""));
      Serial.println(F("Lift the nose of the quadcopter to a 45 degree angle within 10 seconds"));
      //Check axis movement
      check_gyro_axes(2);
    }
    if(error == 0){
      Serial.println(F("OK!"));
      Serial.print(F("Angle detection = "));
      Serial.println(pitch_axis & 0b00000011);
      if(pitch_axis & 0b10000000)Serial.println(F("Axis inverted = yes"));
      else Serial.println(F("Axis inverted = no"));
      Serial.println(F("Put the quadcopter back in its original position"));
      Serial.println(F("Move stick 'nose up' and back to center to continue"));
      check_to_continue();
      
      //Detect the nose right movement
      Serial.println(F(""));
      Serial.println(F(""));
      Serial.println(F("Rotate the nose of the quadcopter 45 degree to the right within 10 seconds"));
      //Check axis movement
      check_gyro_axes(3);
    }
    if(error == 0){
      Serial.println(F("OK!"));
      Serial.print(F("Angle detection = "));
      Serial.println(yaw_axis & 0b00000011);
      if(yaw_axis & 0b10000000)Serial.println(F("Axis inverted = yes"));
      else Serial.println(F("Axis inverted = no"));
      Serial.println(F("Put the quadcopter back in its original position"));
      Serial.println(F("Move stick 'nose up' and back to center to continue"));
      check_to_continue();
    }
  }
   if(error == 0){
    Serial.println(F(""));
    Serial.println(F("==================================================="));
    Serial.println(F("LED test"));
    Serial.println(F("==================================================="));
    digitalWrite(12, HIGH);
    Serial.println(F("The LED should now be lit"));
    Serial.println(F("Move stick 'nose up' and back to center to continue"));
    check_to_continue();
    digitalWrite(12, LOW);
  }
  
  Serial.println(F(""));

  if(error == 0){
    Serial.println(F("======================"));
    Serial.println(F("Final setup check"));
    Serial.println(F("======================"));
    delay(1000);
    if(receiver_check_byte == 0b00001111){
      Serial.println(F("Receiver channels OK"));
    }
    else{
      Serial.println(F("Receiver channel verification failed!!! (ERROR 6)"));
      error = 1;
    }
    delay(1000);
    
    if(gyro_check_byte == 0b00000111){
      Serial.println(F("Gyro axes ok"));
    }
    else{
      Serial.println(F("Gyro exes verification failed!!! (ERROR 7)"));
      error = 1;
    }
  }

  // If all is good, store the information in the EEPROM
  if (error == 0){
    Serial.println(F(""));
    Serial.println(F("========================"));
    Serial.println(F("Storing EEPROM information"));
    Serial.println(F("========================"));
    Serial.println(F("Writing EEPROM"));
    delay(1000);
    Serial.println(F("Done!"));
    // There are 512 bytes in EEPROM. Storing requrired variables one by one.
    // Syntax: EEPROM.write(location, storing_byte)

    // store center stick positions
    // channel_1 = 1500 somethings, which requires 2 bytes, 
    // so storing half bit in 0 and other half in 1
    EEPROM.write(0, center_channel_1 & 0b11111111);
    EEPROM.write(1, center_channel_1 >> 8);
    EEPROM.write(2, center_channel_2 & 0b11111111);
    EEPROM.write(3, center_channel_2 >> 8);
    EEPROM.write(4, center_channel_3 & 0b11111111);
    EEPROM.write(5, center_channel_3 >> 8);
    EEPROM.write(6, center_channel_4 & 0b11111111);
    EEPROM.write(7, center_channel_4 >> 8);
    EEPROM.write(8, high_channel_1 & 0b11111111);
    EEPROM.write(9, high_channel_1 >> 8);
    EEPROM.write(10, high_channel_2 & 0b11111111);
    EEPROM.write(11, high_channel_2 >> 8);
    EEPROM.write(12, high_channel_3 & 0b11111111);
    EEPROM.write(13, high_channel_3 >> 8);
    EEPROM.write(14, high_channel_4 & 0b11111111);
    EEPROM.write(15, high_channel_4 >> 8);
    EEPROM.write(16, low_channel_1 & 0b11111111);
    EEPROM.write(17, low_channel_1 >> 8);
    EEPROM.write(18, low_channel_2 & 0b11111111);
    EEPROM.write(19, low_channel_2 >> 8);
    EEPROM.write(20, low_channel_3 & 0b11111111);
    EEPROM.write(21, low_channel_3 >> 8);
    EEPROM.write(22, low_channel_4 & 0b11111111);
    EEPROM.write(23, low_channel_4 >> 8);
    EEPROM.write(24, channel_1_assign);
    EEPROM.write(25, channel_2_assign);
    EEPROM.write(26, channel_3_assign);
    EEPROM.write(27, channel_4_assign);
    EEPROM.write(28, roll_axis);
    EEPROM.write(29, pitch_axis);
    EEPROM.write(30, yaw_axis);
    EEPROM.write(31, type);
    EEPROM.write(32, gyro_address);

    Serial.println(F("Verify EEPROM data"));
    delay(1000);

        //To make sure evrything is ok, verify the EEPROM data.
    Serial.println(F("Verify EEPROM data"));
    delay(1000);
    if(center_channel_1 != ((EEPROM.read(1) << 8) | EEPROM.read(0)))error = 1;
    if(center_channel_2 != ((EEPROM.read(3) << 8) | EEPROM.read(2)))error = 1;
    if(center_channel_3 != ((EEPROM.read(5) << 8) | EEPROM.read(4)))error = 1;
    if(center_channel_4 != ((EEPROM.read(7) << 8) | EEPROM.read(6)))error = 1;
    
    if(high_channel_1 != ((EEPROM.read(9) << 8) | EEPROM.read(8)))error = 1;
    if(high_channel_2 != ((EEPROM.read(11) << 8) | EEPROM.read(10)))error = 1;
    if(high_channel_3 != ((EEPROM.read(13) << 8) | EEPROM.read(12)))error = 1;
    if(high_channel_4 != ((EEPROM.read(15) << 8) | EEPROM.read(14)))error = 1;
    
    if(low_channel_1 != ((EEPROM.read(17) << 8) | EEPROM.read(16)))error = 1;
    if(low_channel_2 != ((EEPROM.read(19) << 8) | EEPROM.read(18)))error = 1;
    if(low_channel_3 != ((EEPROM.read(21) << 8) | EEPROM.read(20)))error = 1;
    if(low_channel_4 != ((EEPROM.read(23) << 8) | EEPROM.read(22)))error = 1;
    
    if(channel_1_assign != EEPROM.read(24))error = 1;
    if(channel_2_assign != EEPROM.read(25))error = 1;
    if(channel_3_assign != EEPROM.read(26))error = 1;
    if(channel_4_assign != EEPROM.read(27))error = 1;
    
    if(roll_axis != EEPROM.read(28))error = 1;
    if(pitch_axis != EEPROM.read(29))error = 1;
    if(yaw_axis != EEPROM.read(30))error = 1;
    if(type != EEPROM.read(31))error = 1;
    if(gyro_address != EEPROM.read(32))error = 1;
    
    if('J' != EEPROM.read(33))error = 1;
    if('M' != EEPROM.read(34))error = 1;
    if('B' != EEPROM.read(35))error = 1;
  
    if(error == 1)Serial.println(F("EEPROM verification failed!!! (ERROR 5)"));
    else Serial.println(F("Verification done"));
   }

   if (error == 0){
    Serial.println(F("Setup is finished."));
    Serial.println(F("You can now calibrate the esc's and upload the YMFC-AL code."));
   }
   else{
    Serial.println(F("The setup is aborted due to an error."));
   Serial.println(F("Check the Q and A page of the YMFC-AL project on:"));
   Serial.println(F("www.brokking.net for more information about this error."));
   }
  
  
}

// Interrupt Subroutine
// This routine is called every time input 8, 9, 10 or 11 changed state
ISR(PCINT0_vect){
  // current_time datatype should be unsigned long
  current_time = micros(); // Returns the number of microseconds since the Arduino board began running the current program. 
  // This number will overflow (go back to zero), after approximately 70 minutes. 
  // Channel 1 ======================================
  if (PINB & B00000001){               // Is input 8 high? or digitalRead(D8)
    if (last_channel_1 == 0){           // Input 8 changed from 0 to 1?
      last_channel_1 = 1;              // Remember current input state
      timer_1 = current_time;          // ? Set timer_1 to current_time
    } 
  }
  
  else if(last_channel_1 == 1){        // Input 8 is not high meaning if (PINB & B00000000)
                                       // and changed from 1 to 0 meaning last channel = 1
    last_channel_1 = 0;                // Remember current input state
    receiver_input_channel_1 = current_time - timer_1;  // ? Channel 1 is a current_time - timer_1
  }

  //Channel 2=========================================
  if (PINB & B00000010){               // Is input 9 high?
    if (last_channel_2 == 0){          // Is D9 changed from 0 to 1?
      last_channel_2 = 1;              //Remember current input state
      timer_2 = current_time;          // ? Set timer_2 to current_time
    }
  }

  else if(last_channel_2 == 1){                                //Input 9 is not high and changed from 1 to 0
    last_channel_2 = 0;                                        //Remember current input state
    receiver_input_channel_2 = current_time - timer_2;         //? Channel 2 is current_time - timer_2
  }

  //Channel 3=========================================
  if(PINB & B00000100 ){                                       //Is input 10 high?
    if(last_channel_3 == 0){                                   //Input 10 changed from 0 to 1
      last_channel_3 = 1;                                      //Remember current input state
      timer_3 = current_time;                                  //Set timer_3 to current_time
    }
  }
  else if(last_channel_3 == 1){                                //Input 10 is not high and changed from 1 to 0
    last_channel_3 = 0;                                        //Remember current input state
    receiver_input_channel_3 = current_time - timer_3;         //Channel 3 is current_time - timer_3

  }
  //Channel 4=========================================
  if(PINB & B00001000 ){                                       //Is input 11 high?
    if(last_channel_4 == 0){                                   //Input 11 changed from 0 to 1
      last_channel_4 = 1;                                      //Remember current input state
      timer_4 = current_time;                                  //Set timer_4 to current_time
    }
  }
  else if(last_channel_4 == 1){                                //Input 11 is not high and changed from 1 to 0
    last_channel_4 = 0;                                        //Remember current input state
    receiver_input_channel_4 = current_time - timer_4;         //Channel 4 is current_time - timer_4
  }
}



// Intro 
void intro(){
  Serial.println(F("Here goes the introduction, Have Fun!"));
}

void wait_for_receiver(){
  byte zero = 0;
  timer = millis() + 10000;
  while(timer > millis() && zero < 15){ // until 10 seconds && all the inputs are checked
    if (receiver_input_channel_1 < 2100 && receiver_input_channel_1 > 900)zero |= 0b00000001;
    // now zero is 0b00000001
    if (receiver_input_channel_2 < 2100 && receiver_input_channel_2 > 900)zero |= 0b00000010;
    // zero = zero | 0b00000010, so now zero is 0b00000011
    if (receiver_input_channel_3 < 2100 && receiver_input_channel_3 > 900)zero |= 0b00000100;
    // zero = zero | 0b00000100, so now zero is 0b00000111
    if (receiver_input_channel_4 < 2100 && receiver_input_channel_4 > 900)zero |= 0b00001000;
    // zero = zero | 0b00001000, so now zero is 0b00001111 which is 15. HAHA! 
    // This means, until 10 seconds AND until all the receiver inputs: throttle, yaw, 
    // roll and pitch are checked whether they are valid or not (lie within 900 to 2100)
    // this while loop runs.
    delay(500);
    Serial.print(F("."));
  }
  if (zero == 0){ 
    error = 1;
    Serial.println(F("."));
    Serial.println(F("No valid receiver signals found!!! (ERROR 1)"));
  }
  else Serial.println(F("RECEIVERS OK"));
}



// Check if a receiver input value is changing within 30 seconds
void check_receiver_inputs(byte movement){
  byte trigger = 0;
  int pulse_length;
  timer = millis() + 30000;
  while(timer > millis() && trigger == 0){ // until 30 seconds AND until a single receiver
                                           // input is checked no matter which one.
    delay(250); 
    // check input of first receiver: Roll: left wing up is positive implies 
    // the roll stick should receive input > 1750, but, even if the 
    // receiver_input_channel_1 receives input < 1250 (which shows that channel is inverted
    // or right wing up condition it will be FLAGGED in movement code.) 
    if (receiver_input_channel_1 > 1750 || receiver_input_channel_1 < 1250){
      trigger = 1;
      receiver_check_byte |= 0b00000001;
      pulse_length = receiver_input_channel_1;
    }

    // check input of second receiver: pitch
    if (receiver_input_channel_2 > 1750 || receiver_input_channel_2 < 1250){
      trigger = 2;
      receiver_check_byte |= 0b00000010;
      pulse_length = receiver_input_channel_2;
    }

    // check input of third receiver = throttle
    if (receiver_input_channel_3 > 1750 || receiver_input_channel_3 < 1250){
      trigger = 3;
      receiver_check_byte |= 0b00000100;
      pulse_length = receiver_input_channel_3;
    }

    // check input of fourth receiver: YAW
    if (receiver_input_channel_4 > 1750 || receiver_input_channel_4 < 1250){
      trigger = 4; 
      receiver_check_byte |= 0b00001000;
      pulse_length = receiver_input_channel_4;
    }
  }
  /////////////////// ERROR 2  IS HERE /////////////////////////////////
  if (trigger == 0){
    error = 1;
    Serial.println(F("No stick movement detected in the last 30 seconds!!! (ERROR 2)"));
  }
  // Assign the stick to the function.
  else{
    if (movement == 1){ // movement 1 = throttle
      channel_3_assign = trigger;
      // trigger will be 3 = 0b00000011;
      /* below line assigns the latest receiver_input_channel_3 to pulse length in 
      above code corresponding to throttle, and if the latest stick position is 
      not in the center or is not 1500 or is less than 1250 then assign channel 3 
      to another value for showing the assigned pin else channel_3_assign here is 3*/ 
      if (pulse_length < 1250)channel_3_assign = channel_3_assign + 0b10000000; 
      // channel_3_assign = 1 + 128 = 129; 
    }

    if (movement == 2){ // movement 2 = roll
      channel_1_assign = trigger;
      // trigger will be 1 = 0b00000001;
      if (pulse_length < 1250)channel_1_assign += 0b10000000;
      // channel_1_assign = 1 + 128 = 129 = 0b10000001;
    }

    if (movement == 3){ // movement 3 = pitch
      channel_2_assign = trigger;
      // trigger will be 2 = 0b00000010;
      if (pulse_length < 1250)channel_2_assign += 0b10000000;
      // channel_2_assign = 130
    }

    if (movement == 4){ // movement 4 = yaw
      channel_4_assign = trigger;
      if (pulse_length < 1250)channel_4_assign += 0b10000000;
    }
  }
}

// check if the transmitter stiks are in the neutral position
void wait_sticks_zero(){
  byte zero = 0;

  while (zero < 15){
    if (receiver_input_channel_1 < center_channel_1 + 20 && receiver_input_channel_1 > center_channel_1 - 20) |= 0b00000001;
    if (receiver_input_channel_2 < center_channel_2 + 20 && receiver_input_channel_2 > center_channel_2 - 20) |= 0b00000010;
    if (receiver_input_channel_3 < center_channel_3 + 20 && receiver_input_channel_3 > center_channel_3 - 20) |= 0b00000100;
    if (receiver_input_channel_4 < center_channel_4 + 20 && receiver_input_channel_4 > center_channel_4 - 20) |= 0b00001000;
    delay(100);
  }
}

void register_min_max(){
  byte zero = 0;
  low_channel_1 = receiver_input_channel_1;
  low_channel_2 = receiver_input_channel_2;
  low_channel_3 = receiver_input_channel_3;
  low_channel_4 = receiver_input_channel_4;
  // ensuring roll stick is in center, why only roll stick?
  while (receiver_input_channel_1 < center_channel_1 + 20 && receiver_input_channel_1 > center_channel_1 - 20)delay(250);
  Serial.println(F("Measuring endpoints..."));
  while(zero < 15){
    if(receiver_input_channel_1 < center_channel_1 + 20 && receiver_input_channel_1 > center_channel_1 - 20)zero |= 0b00000001;
    if(receiver_input_channel_2 < center_channel_2 + 20 && receiver_input_channel_2 > center_channel_2 - 20)zero |= 0b00000010;
    if(receiver_input_channel_3 < center_channel_3 + 20 && receiver_input_channel_3 > center_channel_3 - 20)zero |= 0b00000100;
    if(receiver_input_channel_4 < center_channel_4 + 20 && receiver_input_channel_4 > center_channel_4 - 20)zero |= 0b00001000;
    // Noting the low channel
    if (receiver_input_channel_1 < low_channel_1)low_channel_1 = receiver_input_channel_1;
    if (receiver_input_channel_2 < low_channel_2)low_channel_2 = receiver_input_channel_2;
    if (receiver_input_channel_3 < low_channel_3)low_channel_3 = receiver_input_channel_3;
    if (receiver_input_channel_4 < low_channel_4)low_channel_4 = receiver_input_channel_4;
    // Noting the high channel
    if(receiver_input_channel_1 > high_channel_1)high_channel_1 = receiver_input_channel_1;
    if(receiver_input_channel_2 > high_channel_2)high_channel_2 = receiver_input_channel_2;
    if(receiver_input_channel_3 > high_channel_3)high_channel_3 = receiver_input_channel_3;
    if(receiver_input_channel_4 > high_channel_4)high_channel_4 = receiver_input_channel_4;
    delay(100);
  }
}

void check_to_continue(){ 
  /* Move stick 'nose up' and back to center to continue. Whenever pitch stick is moved up, 
  check_to_continue is implemented.*/
  byte continue_byte = 0;
  while(continue_byte == 0){
    // last chananel_2_assign was 2 => 0b00000010
    if(channel_2_assign == 0b00000001 && receiver_input_channel_1 > center_channel_1 + 150)continue_byte = 1;
    if(channel_2_assign == 0b10000001 && receiver_input_channel_1 < center_channel_1 - 150)continue_byte = 1;
    if(channel_2_assign == 0b00000010 && receiver_input_channel_2 > center_channel_2 + 150)continue_byte = 1;
    if(channel_2_assign == 0b10000010 && receiver_input_channel_2 < center_channel_2 - 150)continue_byte = 1;
    // 1 && 1 this "if" works.
    if(channel_2_assign == 0b00000011 && receiver_input_channel_3 > center_channel_3 + 150)continue_byte = 1;
    
    if(channel_2_assign == 0b10000011 && receiver_input_channel_3 < center_channel_3 - 150)continue_byte = 1;
    if(channel_2_assign == 0b00000100 && receiver_input_channel_4 > center_channel_4 + 150)continue_byte = 1;
    if(channel_2_assign == 0b10000100 && receiver_input_channel_4 < center_channel_4 - 150)continue_byte = 1;
    delay(100);
  }
  wait_sticks_zero();
}

// function call is search_gyro(0x68, 0x75) == 0x68
byte search_gyro(int gyro_address, int who_am_i){ /*who_am_i for MPU is 0x75 and other is 0x0F*/
  // here gyro_address = 0x68 and who_am_i = 0x75
  Wire.beginTransmission(gyro_address);
  // The slave address of the MPU-60X0 is b110100X which is 7 bits long
  /* WHO_AM_I register is 0x75 which Contains the 6-bit I2C address of the MPU-60X0.
     The Power-On-Reset value of Bit6:Bit1 is 110 100. */
  Wire.write(who_am_i); 
  Wire.endTransmission(); // passes the who_am_i buffer data to slave which is MPU6050
  Wire.requestFrom(gyro_address, 1); // 1 byte is requested = 0b0 110100 0
  timer = millis() + 100;
  while(Wire.available() < 1 && timer > millis()); // wait until 1 byte is received in 100 milliseconds
  lowByte = Wire.read(); // extracts the low-order (rightmost) byte of a variable. 
  // means if a variable is 2 bytes size like int, then lowbyte will read the rightmost 8 bits.which here will be 0b01101000 = 0x68;
  address = gyro_address;
  return lowByte; // returns 0x68
}

void start_gyro(){
  // Setup the MPU-6050
 if(type == 1){
    
    Wire.beginTransmission(address);                             //Start communication with the gyro
    Wire.write(0x6B);                                            //PWR_MGMT_1 register
    Wire.write(0x00);                                            //Set to zero to turn on the gyro
    Wire.endTransmission();                                      //End the transmission
    
    Wire.beginTransmission(address);                             //Start communication with the gyro
    Wire.write(0x6B);                                            //Start reading @ register 28h and auto increment with every read
    Wire.endTransmission();                                      //End the transmission
    Wire.requestFrom(address, 1);                                //Request 1 bytes from the gyro
    while(Wire.available() < 1);                                 //Wait until the 1 byte is received
    Serial.print(F("Register 0x6B is set to:"));
    Serial.println(Wire.read(),BIN);
    
    Wire.beginTransmission(address);                             //Start communication with the gyro
    Wire.write(0x1B);                                            //GYRO_CONFIG register
    Wire.write(0x08);                                            //Set the register bits as 00001000 (500dps full scale)
    Wire.endTransmission();                                      //End the transmission
    
    Wire.beginTransmission(address);                             //Start communication with the gyro (adress 1101001)
    Wire.write(0x1B);                                            //Start reading @ register 28h and auto increment with every read
    Wire.endTransmission();                                      //End the transmission
    Wire.requestFrom(address, 1);                                //Request 1 bytes from the gyro
    while(Wire.available() < 1);                                 //Wait until the 1 byte is received
    Serial.print(F("Register 0x1B is set to:"));
    Serial.println(Wire.read(),BIN);

  }
}

void check_gyro_axes(byte movement){
  byte trigger_axis = 0;
  float gyro_angle_roll, gyro_angle_pitch, gyro_angle_yaw;

  // Reset all axes
  gyro_angle_roll = 0;
  gyro_angle_pitch = 0;
  gyro_angle_yaw = 0;
  gyro_signalen();
  timer = millis() + 10000;
  // for next 10 seconds and roll 
  while(timer > millis() && gyro_angle_roll > -30 && gyro_angle_pitch > -30
  && gyro_angle_pitch < 30 && gyro_angle_yaw > -30 && gyro_angle_yaw < 30){
    gyro_signalen();
    if (type == 1){ // integration is happening here. 
      //gyro rate is being integrated into angles. 
      // the refresh rate of flight controller is 250 Hz or 4 ms.
      // So after dividing raw gyro_roll by 65.5 which is sensitivity 
      // for a full scale range of +/- 500 deg/s, we multiply by 
      // dt = 4ms or divide by frequency = 250Hz 
      // means: angle(1) = angle(0) + (raw_rate/65.5)*0.004
      gyro_angle_roll = gyro_angle_roll + gyro_roll*0.0000611;
      gyro_angle_pitch = gyro_angle_pitch + gyro_pitch*0.0000611;
      gyro_angle_yaw = gyro_angle_yaw + gyro_yaw*0.0000611;
    }
    delayMicroseconds(3700); //Loop is running @ 250Hz. +/- 300us is used for communication with gyro.
  }
  // Assign the moved axis to the corresponding function (pitch, roll, yaw)
  if ((gyro_angle_roll < -30 || gyro_angle_roll > 30) && gyro_angle_pitch > -30 && gyro_angle_pitch < 30 && gyro_angle_yaw > -30 && gyro_angle_yaw < 30){
    gyro_check_byte |= 0b00000001;
    if(gyro_angle_roll < 0)trigger_axis = 0b10000001; // 129
    else trigger_axis = 0b00000001; 
  }
  if((gyro_angle_pitch < -30 || gyro_angle_pitch > 30) && gyro_angle_roll > -30 && gyro_angle_roll < 30 && gyro_angle_yaw > -30 && gyro_angle_yaw < 30){
    gyro_check_byte |= 0b00000010;
    if(gyro_angle_pitch < 0)trigger_axis = 0b10000010; // 
    else trigger_axis = 0b00000010;
  }
  if((gyro_angle_yaw < -30 || gyro_angle_yaw > 30) && gyro_angle_roll > -30 && gyro_angle_roll < 30 && gyro_angle_pitch > -30 && gyro_angle_pitch < 30){
    gyro_check_byte |= 0b00000100;
    if(gyro_angle_yaw < 0)trigger_axis = 0b10000011;
    else trigger_axis = 0b00000011;
  }
  
  if(trigger_axis == 0){
    error = 1;
    Serial.println(F("No angular motion is detected in the last 10 seconds!!! (ERROR 4)"));
  }
  else
  if(movement == 1)roll_axis = trigger_axis;
  if(movement == 2)pitch_axis = trigger_axis;
  if(movement == 3)yaw_axis = trigger_axis;
}

void gyro_signalen(){
  if (type == 1){
    Wire.beginTransmission(address);
    Wire.write(0x43);
    Wire.endTransmission();
    Wire.requestFrom(address, 6);
    while(Wire.available() < 6);
    gyro_roll=Wire.read()<<8|Wire.read();                        //Read high and low part of the angular data
    if(cal_int == 2000)gyro_roll -= gyro_roll_cal;               //Only compensate after the calibration
    gyro_pitch=Wire.read()<<8|Wire.read();                       //Read high and low part of the angular data
    if(cal_int == 2000)gyro_pitch -= gyro_pitch_cal;             //Only compensate after the calibration
    gyro_yaw=Wire.read()<<8|Wire.read();                         //Read high and low part of the angular data
    if(cal_int == 2000)gyro_yaw -= gyro_yaw_cal;                 //Only compensate after the calibration
  }
}
