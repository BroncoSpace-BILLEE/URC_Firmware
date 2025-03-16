// this example is public domain. enjoy! https://learn.adafruit.com/thermocouple/

#include "max6675.h"

int thermoDO = 12; //MISO
int thermoCS = 2; //Slave Select 1
int thermoCLK = 13; //SPI Clock SCK
int thermoCS2 = 3; //Slave Select 2
int thermoCS3 = 4; //Slave Select 3
int thermoCS4 = 5; // Slave Select 4

MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);
MAX6675 thermocouple2(thermoCLK, thermoCS2, thermoDO);
MAX6675 thermocouple3(thermoCLK, thermoCS3, thermoDO);
MAX6675 thermocouple4(thermoCLK, thermoCS4, thermoDO);

void setup() {
  Serial.begin(9600);

  Serial.println("MAX6675 test");
  // wait for MAX chip to stabilize
  delay(500);
}

void loop() {
  // basic readout test, just print the current temp
  Serial.print("C1 = "); 
  Serial.print(thermocouple.readCelsius());
  Serial.print("| C2 = "); 
  Serial.print(thermocouple2.readCelsius());
  Serial.print("| C3 = "); 
  Serial.print(thermocouple3.readCelsius());
  Serial.print("| C4 = "); 
  Serial.println(thermocouple4.readCelsius());

  // For the MAX6675 to update, you must delay AT LEAST 250ms between reads!
  delay(1000);
}