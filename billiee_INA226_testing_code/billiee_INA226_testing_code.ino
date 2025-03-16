#include <Wire.h>
#include <INA226_WE.h>

// Define the I2C addresses for the power monitors
// #define PM1_ADDRESS 0b1000000
// #define PM2_ADDRESS 0b1000001
// #define PM3_ADDRESS 0b1000100

#define PM1_ADDRESS 0x40
#define PM2_ADDRESS 0x41
#define PM3_ADDRESS 0x44


// Initialize the INA226 Devices
INA226_WE PM1 = INA226_WE(&Wire, PM1_ADDRESS);
INA226_WE PM2 = INA226_WE(&Wire, PM2_ADDRESS);
INA226_WE PM3 = INA226_WE(&Wire, PM3_ADDRESS);

// Variables to store values from PM's
float busVoltage1_V = 0.0;
float busVoltage2_V = 0.0;
float busVoltage3_V = 0.0;


void setup() {
  // Initialize Serial for debugging
  Serial.begin(9600);
  
  // Initialize I2C communication
  // Initialize the power monitors
  Serial.println("0");
  // Wire.begin();
  Wire.begin();
  
  // Initialize the power monitors
  Serial.println("A");
  PM1.init();
  PM2.init();
  PM3.init();

  // Increase the average samples to 4 for better accuracy
  Serial.println("B");
  PM1.setAverage(AVERAGE_16);
  PM2.setAverage(AVERAGE_16);
  PM3.setAverage(AVERAGE_16);

  // Set the trigger mode for each device
  Serial.println("C");
  PM1.setMeasureMode(CONTINUOUS);
  PM2.setMeasureMode(CONTINUOUS);
  PM3.setMeasureMode(CONTINUOUS);

  //Let device make one measurement
  Serial.println("D");
  // PM1.waitUntilConversionCompleted();
  // PM2.waitUntilConversionCompleted();
  // PM3.waitUntilConversionCompleted();

}

void loop() {

  // Clear overflow flags
  Serial.println("F");
  PM1.readAndClearFlags();
  PM2.readAndClearFlags();
  PM3.readAndClearFlags();

  // Read and print power data from each monitor
  Serial.println("G");
  busVoltage1_V = PM1.getBusVoltage_V();
  busVoltage2_V = PM2.getBusVoltage_V();
  busVoltage3_V = PM3.getBusVoltage_V();

  // Print values
  Serial.println("H");
  Serial.println(busVoltage1_V);
  Serial.println(busVoltage2_V);
  Serial.println(busVoltage3_V);

  Serial.println("Z");


  delay(1000);
}

void initPowerMonitor(int address) {
  // Code to initialize a power monitor at the given I2C address
  // This will depend on the specific functions provided by your INA266 library
}

void readAndPrintPower(int address) {
  // Code to read power data from a monitor at the given I2C address and print it
  // This will depend on the specific functions provided by your INA266 library
}


