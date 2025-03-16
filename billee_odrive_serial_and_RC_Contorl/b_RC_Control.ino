




/*------------------------- RC Control Functions --------------------*/

// Read the number of a given channel and convert to the range provided.
// If the channel is off, return the default value
int readChannel(int channelInput, int minLimit, int maxLimit, int defaultValue){
  int ch = pulseIn(channelInput, HIGH, 30000);
  if (ch < 100) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}

// Red the channel and return a boolean value
bool RCSwitch(byte channelInput, bool defaultValue){
  int intDefaultValue = (defaultValue)? 100: 0;
  int ch = readChannel(channelInput, 0, 100, intDefaultValue);
  return (ch > 50);
}

void RCPollValueScaled(){

  // Use channl 5 Variable to change the man/min of CH1/2
  ch5Value = readChannel(CH5, 0, 100,0);
  ch1Value = readChannel(CH1, -ch5Value, ch5Value, 0); 
  ch2Value = readChannel(CH2, -ch5Value, ch5Value, 0);
  ch6Value = RCSwitch(CH6, false);
}

void  RCPollValueRaw(){
  ch5Value = readChannel(CH5, 0, 100,0);
  ch1Value = readChannel(CH1, -100, 100, 0); 
  ch2Value = readChannel(CH2, -100, 100, 0);
  ch6Value = RCSwitch(CH6, false);
}

void RCTankDrive(){

  RCPollValueScaled();

  if(ch6Value){


    if(ch1Value==0){
      // Set the velocity for all motors - FWD/Reverse
      odrv1.setVelocity(-ch2Value);
      odrv2.setVelocity(-ch2Value);
      odrv3.setVelocity(-ch2Value);
      odrv4.setVelocity(ch2Value);
      odrv5.setVelocity(ch2Value);
      odrv6.setVelocity(ch2Value);
    }


    if(ch2Value==0){
    // Set the velocity for all motors - CW/CCW
    odrv1.setVelocity(-ch1Value);
    odrv2.setVelocity(-ch1Value);
    odrv3.setVelocity(-ch1Value);
    odrv4.setVelocity(-ch1Value);
    odrv5.setVelocity(-ch1Value);
    odrv6.setVelocity(-ch1Value);
    }

  }

  if(!ch6Value){
    // Stop all motors
    odrv1.setVelocity(0);
    odrv2.setVelocity(0);
    odrv3.setVelocity(0);
    odrv4.setVelocity(0);
    odrv5.setVelocity(0);
    odrv6.setVelocity(0);
  }


}

void RCPrintDebug(){

  // RCPollValueRaw();

  Serial.print("Ch1: ");
  Serial.print(ch1Value);
  Serial.print(" Ch2: ");
  Serial.print(ch2Value);
  Serial.print(" Ch5: ");
  Serial.print(ch5Value);
  Serial.print(" Ch6: ");
  Serial.println(ch6Value);
}

void RCSetupPins(){
   /*--------- RC Control Setup ----------*/
  pinMode(CH1,INPUT); //read PWM from receiver
  pinMode(CH2,INPUT); //read PWM from receiver
  pinMode(CH5,INPUT); //read PWM from receiver
  pinMode(CH6,INPUT); //read PWM from receiver
}
