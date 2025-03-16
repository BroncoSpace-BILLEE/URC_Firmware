#include <SD.h>
#include "max6675.h"
#include <Wire.h>
#include <INA226_WE.h>
#include <Adafruit_NeoPixel.h>
#include "RTClib.h"


/******************************************************************************************************
The following code will be uploaded to a teensy 4.1 and will:
- Track bus voltages of rover
- Track four temperature probes in degree C
- Update local SD Card for data logging
- Track bus voltage battery percentages
- Control LED Array from RGB strip. 


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

COLOR BIT
0 -> Clear LED
1 -> Breathing/Flashing RED LED
2 -> Breathing/Flashing Green LED
3 -> Breathing/Flashing Blue LED

DATA SERIAL PRINT FORMAT

THERMOCOUPLE_1,THERMOCOUPLE_2,THERMOCOUPLE_3,THERMOCOUPLE_4,BUS_VOLTAGE_1,BUS_VOLTAGE_2,BUS_VOLTAGE_3,VBUS1_PCT,VBUS2_PCT,VBUS3_PCT,SDGood,RTCGOOD,Colot_bit


*******************************************************************************************************/
//DEBUG Flags
// #define DEBUG_PRINT 0

// SD Card Setup
File myFile;
int SDGood = 0;  // Boolean to track if sd card is ready
// const char* fileName = "PM_DATA_LOG.txt";
String baseFilename = "PM_DATA_LOG_";
String fullFilename = "";

// Thermocouple Globals
int thermoDO = 12;   //MISO
int thermoCLK = 13;  //SPI Clock SCK
int thermoCS = 2;    //Slave Select 1
int thermoCS2 = 3;   //Slave Select 2
int thermoCS3 = 4;   //Slave Select 3
int thermoCS4 = 5;   // Slave Select 4

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

float const LIPO_DRIVE_VBUS_MIN = 12.80;         // Volts
float const LIPO_DRIVE_VBUS_MAX = 16.80;         // Volts
float const LIPO_LOGIC_LOWER_VBUS_MIN = 19.20;   // Volts
float const LIPO_LOGIC_LOWER_VBUS_MAX = 25.20;   // Volts
float const LIPO_LOGIC_UPPER_VBUS_MIN = 38.40;   // Volts
float const LIPO_LOGIC_UPPER_VBUS_MAX = 50.40;   // Volts
float const BUS_VOLTAGE_3_DIVIDER_RATIO = 0.39;  // From voltage divider on Protoboard.

float temp1_C = 0.0;
float temp2_C = 0.0;
float temp3_C = 0.0;
float temp4_C = 0.0;

// LED Neo pixel setup
#define PIN_NEO_PIXEL 33  // Arduino pin that connects to NeoPixel
#define NUM_PIXELS 2     // The number of LEDs (pixels) on NeoPixel

#define NUM_REPEAT 1    // Number of breathes
#define BRIGHTNESS 100  //LED Brightness
int color_bit = 0;
String serialInput = "0\n";  // hold serial unput

Adafruit_NeoPixel NeoPixel(NUM_PIXELS, PIN_NEO_PIXEL, NEO_RGB + NEO_KHZ800);


// Variables for time-based events
#define SENSOR_READ_STATE_TIMEOUT 300      // mSeconds to wait until sampleing senosrs
#define SD_CARD_PRINT_STATE_TIMEOUT 5000   // mSeconds to wait until SD Card update
#define SERIAL_PRINT_STATE_TIMEOUT 1000    // mSeconds to print values to serial
#define SD_CARD_STATUS_CHECK_TIMEOUT 2000  // mSeconds to wait to check SD card is availiale

unsigned long sensorReadPreviousTime = 0;    // Time accumulated at end of loop
unsigned long sdCardPrintPreviousTime = 0;   // Time accumulated at end of loop
unsigned long sdCardStatusPreviousTime = 0;  // Time accumulated at end of loop
unsigned long serialPrintPreviousTime = 0;   // Time accumulated at end of loop

unsigned long currentTime = 0;  //Time at the beginning of loop

// Objects for RTC
RTC_PCF8523 rtc;
int RTCGOOD = 0;


void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ;  // wait for serial port to connect.
  }

  // Wire.begin();
  Wire.begin();
 
  //Init RTC - Do this first
  // Using RTC without init first, will cause hanging
  initRTC();

  //Grab Current time
  DateTime now =  rtc.now();
  fullFilename =  baseFilename +
                  String(now.year(), DEC) + "-" +
                  String(now.month(), DEC) + "-" +
                  String(now.day(), DEC) + "_" +
                  String(now.hour(), DEC) + "-" +
                  String(now.minute(), DEC) + "-" +
                  String(now.second(), DEC) + ".txt";

  #ifdef DEBUG_PRINT
    Serial.println(fullFilename.c_str());
  #endif

  //Initialize The SD Card.
  initSDCard();


  //init INA devices
  initPowerMonitor();


  // INITIALIZE NeoPixel strip object (REQUIRED)
  NeoPixel.begin();

  delay(1000);

  //Set brightness and ensure neopixel is off
  NeoPixel.show();
  NeoPixel.clear();                    // set all pixel colors to 'off'. It only takes effect if pixels.show() is called
  NeoPixel.setBrightness(BRIGHTNESS);  // a value from 0 to 255
}

void loop() {
  //Save the time since turn-on
  currentTime = millis();

  if ((currentTime - sensorReadPreviousTime) > SENSOR_READ_STATE_TIMEOUT) {

    // Update and print Temperature Values
    // For the MAX6675 to update, you must delay AT LEAST 250ms between reads!
    sampleTempC();

    // Update voltage values
    sampleBusVoltage();

    estimateBatteryLife();

    sensorReadPreviousTime = currentTime;
  }

  if ((currentTime - sdCardPrintPreviousTime) > SD_CARD_PRINT_STATE_TIMEOUT) {

    // Update and print to SD Card
    updateSDCard();

    sdCardPrintPreviousTime = currentTime;
  }

  if ((currentTime - serialPrintPreviousTime) > SERIAL_PRINT_STATE_TIMEOUT) {

    //SD Status update and Newline
    printToSerial();

    serialPrintPreviousTime = currentTime;
  }

  // Parse in serial input to extract color bit to set RGB animation
  if (Serial.available() > 1) {

    serialInput = Serial.readStringUntil('\n');

    sscanf(serialInput.c_str(), "%i", &color_bit);

    #ifdef DEBUG_PRINT
      Serial.print("Received Color_bit: ");
      Serial.println(color_bit);
    #endif

    //Input validation
    if (color_bit != 1 && color_bit != 2 && color_bit != 3) {
      color_bit = 0;
    }
  }

  //Based on color bit, select the RGB animation
  colorBitHandler();
}


void initRTC(){
  if (!rtc.begin()) {
    RTCGOOD = 0;
  } else {
    RTCGOOD = 1;
  }

  if (! rtc.initialized() || rtc.lostPower()) {

  #ifdef DEBUG_PRINT  
    Serial.println("RTC is NOT initialized, let's set the time!");
  #endif
  // When time needs to be set on a new device, or after a power loss, the
  // following line sets the RTC to the date & time this sketch was compiled
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  // This line sets the RTC with an explicit date & time, for example to set
  // January 21, 2014 at 3am you would call:
  // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  //
  // Note: allow 2 seconds after inserting battery or applying external power
  // without battery before calling adjust(). This gives the PCF8523's
  // crystal oscillator time to stabilize. If you call adjust() very quickly
  // after the RTC is powered, lostPower() may still return true.
}
}

void sampleBusVoltage() {
  // Clear overflow flags
  PM1.readAndClearFlags();
  PM2.readAndClearFlags();
  PM3.readAndClearFlags();

  // Read and print power data from each monitor
  busVoltage1_V = PM1.getBusVoltage_V();
  busVoltage2_V = PM2.getBusVoltage_V();
  busVoltage3_V = PM3.getBusVoltage_V();
}

void estimateBatteryLife() {
  //Calculate battery life and print
  LipoCapacity1_pct = (busVoltage1_V - LIPO_DRIVE_VBUS_MIN) / (LIPO_DRIVE_VBUS_MAX - LIPO_DRIVE_VBUS_MIN);
  LipoCapacity2_pct = (busVoltage2_V - LIPO_LOGIC_LOWER_VBUS_MIN) / (LIPO_LOGIC_LOWER_VBUS_MAX - LIPO_LOGIC_LOWER_VBUS_MIN);
  LipoCapacity3_pct = (busVoltage3_V - LIPO_LOGIC_UPPER_VBUS_MIN * BUS_VOLTAGE_3_DIVIDER_RATIO) / ((LIPO_LOGIC_UPPER_VBUS_MAX - LIPO_LOGIC_UPPER_VBUS_MIN) * BUS_VOLTAGE_3_DIVIDER_RATIO);
}

void sampleTempC() {
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

void initSDCard() {

  //Only run function if the SDGOOD is false
  if (SDGood) {
    return;
  }

  //Initialize The SD Card.
  if (!SD.begin(BUILTIN_SDCARD)) {
    SDGood = 0;  // Error with SD Card.
    return;
  } else {
    // open the file for write at end like the "Native SD library"
    myFile = SD.open(fullFilename.c_str(), FILE_WRITE);

    // Check that the SD Card can be used so that data is saved.
    if (myFile) {
      SDGood = 1;  // NO Error with SD Card. Logging date now.

    } else {
      #ifdef DEBUG_PRINT
        Serial.print("$ERROR: File not opened");
      #endif
      SDGood = 0;  // Error with SD Card.
    }

  }
}


void printToSerial() {
  // basic readout test, just print the current temp
  Serial.print(temp1_C);  //---> Thermocouple 1
  Serial.print(",");
  Serial.print(temp2_C);  //---> Thermocouple 2
  Serial.print(",");
  Serial.print(temp3_C);  //---> Thermocouple 3
  Serial.print(",");
  Serial.print(temp4_C);  //---> Thermocouple 4

  // Code to read power data from a monitor at the given I2C address and print it
  Serial.print(",");
  Serial.print(busVoltage1_V);  //---> DRIVE Bus Voltage (14.8V Nominal)
  Serial.print(",");
  Serial.print(busVoltage2_V);  //---> LOGIC Bus Voltage (22.4V Nominal)
  Serial.print(",");
  Serial.print(busVoltage3_V);  //---> LOGIC Bus Voltage (44.8V Nominal)

  // Estimate battery life of lipos.
  Serial.print(",");
  Serial.print(LipoCapacity1_pct);  //---> DRIVE Bus Voltage (14.8V Nominal) Battery Life
  Serial.print(",");
  Serial.print(LipoCapacity2_pct);  //---> LOGIC Bus Voltage (22.4V Nominal) Battery Life
  Serial.print(",");
  Serial.print(LipoCapacity3_pct);  //---> LOGIC Bus Voltage (44.8V Nominal) Battery Life

  // Print error status
  Serial.print(",");
  Serial.print(SDGood);

  // Print the status of RTC
  Serial.print(",");
  Serial.print(RTCGOOD);

  // Print the current ColotBit Value
  Serial.print(",");
  Serial.print(color_bit);

  //Add newline for easier parsing
  Serial.println("");
}


void updateSDCard() {

  //Check if SD Card Initialized
  if (!SDGood) {

    #ifdef DEBUG_PRINT
      Serial.println("$ERROR: No SD Card");
    #endif

    return;  // leave the function.
  }

  if(!RTCGOOD){

    #ifdef DEBUG_PRINT
      Serial.println("$ERROR: No Real-Time-Clock");
    #endif

    return; // Don't record time
  }

  //Grab current time
  DateTime now = rtc.now();

  // Send data to SD Card text file
  myFile = SD.open(fullFilename.c_str(), FILE_WRITE);

  if (myFile) {

    //Print a timestamp ----> hour:minute:second
    myFile.print(now.hour(), DEC); 
    myFile.print(":");
    myFile.print(now.minute(), DEC);
    myFile.print(":");
    myFile.print(now.second(), DEC);
    myFile.print(",");
    
    //Print the current temp
    myFile.print(temp1_C);  //---> Thermocouple 1
    myFile.print(",");
    myFile.print(temp2_C);  //---> Thermocouple 2
    myFile.print(",");
    myFile.print(temp3_C);  //---> Thermocouple 3
    myFile.print(",");
    myFile.print(temp4_C);  //---> Thermocouple 4

    // Code to read power data from a monitor at the given I2C address and print it
    myFile.print(",");
    myFile.print(busVoltage1_V);  //---> DRIVE Bus Voltage (14.8V Nominal)
    myFile.print(",");
    myFile.print(busVoltage2_V);  //---> LOGIC Bus Voltage (22.4V Nominal)
    myFile.print(",");
    myFile.print(busVoltage3_V);  //---> LOGIC Bus Voltage (44.8V Nominal)

    // Estimate battery life of lipos.
    myFile.print(",");
    myFile.print(LipoCapacity1_pct);  //---> DRIVE Bus Voltage (14.8V Nominal) Battery Life
    myFile.print(",");
    myFile.print(LipoCapacity2_pct);  //---> LOGIC Bus Voltage (22.4V Nominal) Battery Life
    myFile.print(",");
    myFile.print(LipoCapacity3_pct);  //---> LOGIC Bus Voltage (44.8V Nominal) Battery Life

    myFile.println("");

    myFile.close();

  } else {
    SDGood = 0;  // Error with accessing SD Card
  }
}
