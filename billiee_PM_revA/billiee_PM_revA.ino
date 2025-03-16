#include <SD.h>
#include "max6675.h"
#include <Wire.h>
#include <INA226_WE.h>
#include <Adafruit_NeoPixel.h>


/******************************************************************************************************
The following code will be uploaded to a teensy 4.1 and will:
- Track bus voltages of rover
- Track four temperature probes in degree C
- Update local SD Card for data logging
- Track bus voltage battery percentages
- Controll LED Array from RGB strip. 


LIPO Battery range
Min -> 3.2V
Nominal -> 3.7V
Max -> 4.2V


BUS_VOLTAGE_1: Tracks the drivetrain voltage
- Minimum: 12.80V
- Nominal: 14.8V
- Max: 16.80V

BUS_VOLTAGE_2
- Minimum: 19.20V
- Nominal: 22.20V
- Max: 25.20V

BUS_VOLTAGE_3 ( Contains a voltage divider of 1/2.55 or 0.39xVBUS)
- Minimum: 38.40V
- Nominal: 44.40V
- Max: 50.40V



DATA SERIAL PRINT FORMAT

THERMOCOUPLE_1,THERMOCOUPLE_2,THERMOCOUPLE_3,THERMOCOUPLE_4,BUS_VOLTAGE_1,BUS_VOLTAGE_2,BUS_VOLTAGE_3,VBUS1_PCT.VBUS2_PCT.VBUS3_PCT,SDGood


*******************************************************************************************************/

// SD Card Setup
File myFile;
int SDGood = 0; // Boolean to track if sd card is ready
const char* fileName = "PM_DATA_LOG.txt";

// Thermocouple Globals
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

// INA globals
#define PM1_ADDRESS 0x40
#define PM2_ADDRESS 0x41
#define PM3_ADDRESS 0x44
 

// Initialize the INA226 Devices
INA226_WE PM1 = INA226_WE(&Wire, PM1_ADDRESS);
INA226_WE PM2 = INA226_WE(&Wire, PM2_ADDRESS);
INA226_WE PM3 = INA226_WE(&Wire, PM3_ADDRESS);

// Variables to store values from PM's and Thermocouples
float busVoltage1_V = 0.0;
float busVoltage2_V = 0.0;
float busVoltage3_V = 0.0;

float LipoCapacity1_pct = 0.0;
float LipoCapacity2_pct = 0.0;
float LipoCapacity3_pct = 0.0;

float const LIPO_DRIVE_VBUS_MIN =  12.80;       // Volts
float const LIPO_DRIVE_VBUS_MAX =  16.80;       // Volts
float const LIPO_LOGIC_LOWER_VBUS_MIN =  19.20; // Volts
float const LIPO_LOGIC_LOWER_VBUS_MAX =  25.20; // Volts
float const LIPO_LOGIC_UPPER_VBUS_MIN =  38.40; // Volts
float const LIPO_LOGIC_UPPER_VBUS_MAX =  50.40; // Volts
float const BUS_VOLTAGE_3_DIVIDER_RATIO = 0.39; // From voltage divider on Protoboard. 

float temp1_C = 0.0;
float temp2_C = 0.0;
float temp3_C = 0.0;
float temp4_C = 0.0;

// LED PWM Pins
#define RED_LED_PIN 10
#define GREEN_LED_PIN 9
#define BLUE_LED_PIN 8

#define MAX_LED_BRIGHTNESS 255
#define SET_LED_BRIGHTNESS 100
#define LED_OFF 0 




void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect.
  }
 
  //Initialize The SD Card. 
  initSDCard();
 
  // Wire.begin();
  Wire.begin();

  //init INA devices
  initPowerMonitor();

  //Setup RGB pins
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(BLUE_LED_PIN, OUTPUT);

}

void loop() {

  // Update and print Temperature Values
  sampleTempC();

  // Update and print voltage values
  sampleBusVoltage();

  estimateBatteryLife();
 
  // Update and print to SD Card
  updateSDCard();
  
  //SD Status update and Newline  
  printToSerial();

  // For the MAX6675 to update, you must delay AT LEAST 250ms between reads!
  delay(1000);


}

void RGBOff()
{
  analogWrite(BLUE_LED_PIN, LED_OFF);
  analogWrite(GREEN_LED_PIN, LED_OFF);
  analogWrite(RED_LED_PIN, LED_OFF);
}

void flashPurpleLED()
{

  RGBOff();
  // Fade ON and OFF for PURPLE
  for (int i = 0; i <= 255; i++) {
      analogWrite(RED_LED_PIN, i);
      analogWrite(BLUE_LED_PIN, i);

      delay(1);
  }
  for (int i = 255; i >= 0; i--) {
      analogWrite(RED_LED_PIN, i);
      analogWrite(BLUE_LED_PIN, i);
      delay(1);
  }

  delay(150);
}

void constantGreenLED()
{
  RGBOff();
  analogWrite(GREEN_LED_PIN, SET_LED_BRIGHTNESS);
}

void constantBlueLED()
{
  RGBOff();
  analogWrite(BLUE_LED_PIN, SET_LED_BRIGHTNESS);
}

void constantRedLED()
{
  RGBOff();
  analogWrite(RED_LED_PIN, SET_LED_BRIGHTNESS);
}

void breathRedLED()
{
  RGBOff();
  // Fade ON and OFF for RED
  for (int i = 100; i <= 255; i++) {
      analogWrite(RED_LED_PIN, i);
      delay(5);
  }
  for (int i = 255; i >= 100; i--) {
      analogWrite(RED_LED_PIN, i);
      delay(5);
  }
}

void breathBlueLED()
{
  RGBOff();
  // Fade ON and OFF for BLUE
  for (int i = 100; i <= 255; i++) {
      analogWrite(BLUE_LED_PIN, i);
      delay(5);
  }
  for (int i = 255; i >= 100; i--) {
      analogWrite(BLUE_LED_PIN, i);
      delay(5);
  }

}

void flashRedLED()
{
  RGBOff();
  // Fade ON and OFF for GREEN
  for (int i = 0; i <= 255; i++) {
      analogWrite(RED_LED_PIN, i);
      delay(1);
  }
  for (int i = 255; i >= 0; i--) {
      analogWrite(RED_LED_PIN, i);
      delay(1);
  }

  delay(150);
}

void flashGreenLED()
{
  RGBOff();
  // Fade ON and OFF for GREEN
  for (int i = 0; i <= 255; i++) {
      analogWrite(GREEN_LED_PIN, i);
      delay(1);
  }
  for (int i = 255; i >= 0; i--) {
      analogWrite(GREEN_LED_PIN, i);
      delay(1);
  }

  delay(150);
}

void sampleBusVoltage()
{
  // Clear overflow flags
  PM1.readAndClearFlags();
  PM2.readAndClearFlags();
  PM3.readAndClearFlags();

  // Read and print power data from each monitor
  busVoltage1_V = PM1.getBusVoltage_V();
  busVoltage2_V = PM2.getBusVoltage_V();
  busVoltage3_V = PM3.getBusVoltage_V();
}

void estimateBatteryLife()
{
  //Calculate battery life and print
  LipoCapacity1_pct = (busVoltage1_V - LIPO_DRIVE_VBUS_MIN) / (LIPO_DRIVE_VBUS_MAX - LIPO_DRIVE_VBUS_MIN);
  LipoCapacity2_pct = (busVoltage2_V - LIPO_LOGIC_LOWER_VBUS_MIN) / (LIPO_LOGIC_LOWER_VBUS_MAX - LIPO_LOGIC_LOWER_VBUS_MIN);
  LipoCapacity3_pct = (busVoltage3_V - LIPO_LOGIC_UPPER_VBUS_MIN* BUS_VOLTAGE_3_DIVIDER_RATIO) / ( (LIPO_LOGIC_UPPER_VBUS_MAX-LIPO_LOGIC_UPPER_VBUS_MIN) * BUS_VOLTAGE_3_DIVIDER_RATIO);
}

void sampleTempC()
{
  // Thermo Test Code
  temp1_C = thermocouple.readCelsius();
  temp2_C = thermocouple2.readCelsius();
  temp3_C = thermocouple3.readCelsius();
  temp4_C = thermocouple4.readCelsius();
}


void initPowerMonitor() {
  // Code to initialize a power monitor at the given I2C address
  // Initialize the power monitors
  PM1.init();
  PM2.init();
  PM3.init();

  // Increase the average samples to 4 for better accuracy
  PM1.setAverage(AVERAGE_16);
  PM2.setAverage(AVERAGE_16);
  PM3.setAverage(AVERAGE_16);

  // Set the trigger mode for each device
  PM1.setMeasureMode(CONTINUOUS);
  PM2.setMeasureMode(CONTINUOUS);
  PM3.setMeasureMode(CONTINUOUS);

  //Let device make one measurement
  PM1.waitUntilConversionCompleted();
  PM2.waitUntilConversionCompleted();
  PM3.waitUntilConversionCompleted();
  
}

void initSDCard()
{
  //Initialize The SD Card. 
  if (!SD.begin(BUILTIN_SDCARD)) 
  {
    SDGood = 0; // Error with SD Card.
    return;
  } else {
    // open the file for write at end like the "Native SD library"
    myFile = SD.open(fileName, FILE_WRITE);

    // Check that the SD Card can be used so that data is saved. 
    if (myFile) 
    {
      SDGood = 1; // NO Error with SD Card. Logging date now. 

      //Erase File
      SD.remove(fileName);


    } else {
      SDGood = 0; // Error with SD Card.
    }

    // open the file for write at end like the "Native SD library"
    myFile = SD.open(fileName, FILE_WRITE);

    // Check that the SD Card can be used so that data is saved. 
    if (myFile) 
    {
      SDGood = 1; // NO Error with SD Card. Logging date now. 
    } else {
      SDGood = 0; // Error with SD Card.
    }
  
  }
}


void printToSerial()
{
  // basic readout test, just print the current temp
  Serial.print(temp1_C);    //---> Thermocouple 1
  Serial.print(","); 
  Serial.print(temp2_C);    //---> Thermocouple 2
  Serial.print(","); 
  Serial.print(temp3_C);    //---> Thermocouple 3
  Serial.print(","); 
  Serial.print(temp4_C);    //---> Thermocouple 4

  // Code to read power data from a monitor at the given I2C address and print it
  Serial.print(",");
  Serial.print(busVoltage1_V); //---> DRIVE Bus Voltage (14.8V Nominal)
  Serial.print(",");
  Serial.print(busVoltage2_V); //---> LOGIC Bus Voltage (22.4V Nominal)
  Serial.print(",");
  Serial.print(busVoltage3_V); //---> LOGIC Bus Voltage (44.8V Nominal)

  // Estimate battery life of lipos. 
  Serial.print(",");
  Serial.print(LipoCapacity1_pct); //---> DRIVE Bus Voltage (14.8V Nominal) Battery Life
  Serial.print(",");
  Serial.print(LipoCapacity2_pct);  //---> LOGIC Bus Voltage (22.4V Nominal) Battery Life
  Serial.print(",");
  Serial.print(LipoCapacity3_pct); //---> LOGIC Bus Voltage (44.8V Nominal) Battery Life

  // Print error status
  Serial.print(",");
  Serial.print(SDGood);

  Serial.println("");
}



void updateSDCard()
{

  //Check if SD Card Initialized
  if(!SDGood)
  {
    return; // leave the function. 
  }

  // Send data to SD Card text file
  myFile = SD.open(fileName, FILE_WRITE);

  if(myFile)
  {

    // basic readout test, just print the current temp
    myFile.print(temp1_C);    //---> Thermocouple 1
    myFile.print(","); 
    myFile.print(temp2_C);    //---> Thermocouple 2
    myFile.print(","); 
    myFile.print(temp3_C);    //---> Thermocouple 3
    myFile.print(","); 
    myFile.print(temp4_C);    //---> Thermocouple 4

    // Code to read power data from a monitor at the given I2C address and print it
    myFile.print(",");
    myFile.print(busVoltage1_V); //---> DRIVE Bus Voltage (14.8V Nominal)
    myFile.print(",");
    myFile.print(busVoltage2_V); //---> LOGIC Bus Voltage (22.4V Nominal)
    myFile.print(",");
    myFile.print(busVoltage3_V); //---> LOGIC Bus Voltage (44.8V Nominal)

    // Estimate battery life of lipos. 
    myFile.print(",");
    myFile.print(LipoCapacity1_pct); //---> DRIVE Bus Voltage (14.8V Nominal) Battery Life
    myFile.print(",");
    myFile.print(LipoCapacity2_pct);  //---> LOGIC Bus Voltage (22.4V Nominal) Battery Life
    myFile.print(",");
    myFile.print(LipoCapacity3_pct); //---> LOGIC Bus Voltage (44.8V Nominal) Battery Life
    
    myFile.println("");
 
    myFile.close();

  } else
    {
      SDGood = 0; // Error with accessing SD Card
    }
}
