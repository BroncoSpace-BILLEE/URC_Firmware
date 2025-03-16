

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
//input 19
}

void loop() {
  // put your main code here, to run repeatedly:
  int sensorValue = analogRead(A5);
  float voltage = sensorValue * (5.0 / 1550.0);
  // print out the value you read:
  Serial.println(voltage);
//int b;
//b = digitalRead(A5);
  delay(200);
}
