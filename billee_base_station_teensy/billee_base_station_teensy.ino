int incomingByte = 0;
char incomingChar;

void setup() {
  // put your setup code here, to run once:

  Serial8.begin(9600);
  Serial.begin(115200);

  Serial.println("DPGS RTCM TEST");

}

void loop() {
  // put your main code here, to run repeatedly:


  while (Serial8.available() > 0) {
    incomingByte = Serial8.read();
    incomingChar = Serial8.read();

    // Serial.print(incomingByte);
    Serial.print(incomingChar);
   
  }
  // Serial.println();
  delay(11);

}
