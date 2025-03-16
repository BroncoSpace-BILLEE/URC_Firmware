/*

Serial Send Format expected: "UV_LED_BIT\n"
Feedback format: "UV_LED_BIT\n"


UV_LED_BIT: determines whether the UV LED is on. A hardcoded timeout is 
used to turn off the LED. 


*/




//DEBUG Flags
// #define DEBUG_PRINT 1
#define TEENSY_LED 13
#define UV_LED_RELAY_PIN 37

//Constants for time-based events
#define SERIAL_PRINT_STATE_TIMEOUT 1000  //mSeconds to wait before printing
#define UV_OFF_STATE_TIMEOUT 10000        //mSeconds to turn off UV led due to 
#define OVERALL_LOOP_DELAY 5             //mSeconds for simple delay loop

unsigned long serialPrintPreviousTime = 0;  // Time accumulated at end of loop
unsigned long UVOnPreviousTime = 0;         //Time that UV LED was ON for
unsigned long currentTime = 0;              //Time at the beginning of loop

int UV_LED_BIT = 0; // Hold the LED Set bit
bool isUvON = true; // Hold the previous LED bit state

String serialInput = "0\n"; // Hold Serial input

void setup() {

  Serial.begin(115200);
  while (!Serial) {
    ;  // wait for serial port to connect.
  }

  // setup UV LED pin and turn teensy LED to indicate status
  pinMode(UV_LED_RELAY_PIN,OUTPUT);

  pinMode(TEENSY_LED, OUTPUT);

  digitalWrite(TEENSY_LED, HIGH);

}

void loop() {

  //Save the time since turn-on
  currentTime = millis();

  //Handle time-based functions
  if((currentTime - serialPrintPreviousTime) > SERIAL_PRINT_STATE_TIMEOUT){

     #ifdef DEBUG_PRINT
      Serial.println("Serial print Timeout!");
    #endif

    serialFeedback();

    serialPrintPreviousTime = currentTime;
  }

  if(isUvON){
    if((currentTime - UVOnPreviousTime) > UV_OFF_STATE_TIMEOUT){
      // Turn off the UV LED to prevent thermal overheat!!!
      #ifdef DEBUG_PRINT
        Serial.println("UV on timeout");
      #endif

      UV_LED_BIT = 0;

      UVOnPreviousTime = currentTime;

    }
  }

  //Handle Serial input
  if(Serial.available() > 1){

    #ifdef DEBUG_PRINT
      Serial.print("serial received");
    #endif

    //Extract the UV bit and perform input validation
    serialInput = Serial.readStringUntil('\n');

    sscanf(serialInput.c_str(), "%i", &UV_LED_BIT);

    #ifdef DEBUG_PRINT
      Serial.print("Received UV Bit: ");
      Serial.println(UV_LED_BIT);
    #endif

    if(UV_LED_BIT != 1 && UV_LED_BIT != 0){
      UV_LED_BIT = 0;

      #ifdef DEBUG_PRINT
        Serial.println("Invalid UV bit!");
      #endif

    }

    

  }

  //Handle UV Bit input
  handleUVBit();

  delay(OVERALL_LOOP_DELAY);

}

void handleUVBit(){
  // Handle UV LED bit
  if(UV_LED_BIT){
    digitalWrite(UV_LED_RELAY_PIN, HIGH);
    
    // Change UV light status and update timeing
    if(isUvON == false){
      isUvON = true;

      UVOnPreviousTime = currentTime;
    }
   
  } else {
    digitalWrite(UV_LED_RELAY_PIN, LOW);

    isUvON = false;

  }

}

void serialFeedback(){
  //Provide feedback via serial
  Serial.print(UV_LED_BIT);

  Serial.println();
}