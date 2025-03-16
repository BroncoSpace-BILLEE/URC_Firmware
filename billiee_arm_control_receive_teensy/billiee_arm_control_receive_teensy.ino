#include <Arduino.h>
#include "ODriveCAN.h"
#include <FlexCAN_T4.h>
#include "ODriveFlexCAN.hpp"
#include <SoftwareSerial.h>
#include <Tic.h>

/*------------------------------DEFINING PARAMETERS------------------------------*/

// Definition for 1st Axis (ODESC V4.2 CAN BUS)
#define ODRIVE_VEL 12
#define CAN_BAUDRATE 500000
#define ODRV0_NODE_ID 2
#define ODRV1_NODE_ID 3

// Definitions for 2nd Axis (RoboClaw communication)
#define ROBOCLAW_ADDRESS 0x80 // RoboClaw address (128 in decimal)
#define MOTOR_FORWARD 0 // Command byte for driving the motor forward
#define MOTOR_BACKWARD 1 // Command byte for driving the motor backward
SoftwareSerial roboclawSerial(15,14);

// Definitions for 3rd Axis (Digital communication)
#define PUL 36
#define DIR 38

// Definitions for Gripper (TIC communication)
SoftwareSerial ticSerial(21, 20); // RX, TX for Tic communication
TicSerial tic(ticSerial);
#define MAX_VELOCITY 2000000 // Maximum velocity for the stepper motor

/* Teensy with built-in CAN interface */
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can_0;

/*------------------------------ODRIVE (ODESC V4.2) SETUP AND FUNCTIONS------------------------------*/
// Setup Step
struct ODriveStatus; // Hack to prevent Teensy compile error
unsigned long startTime = millis();
bool setupCan() {
  // CAN 0 Setup (1st Axis)
  can_0.begin();
  can_0.setBaudRate(CAN_BAUDRATE);
  can_0.setMaxMB(16);
  can_0.enableFIFO();
  can_0.enableFIFOInterrupt();
  can_0.onReceive(receiveCallback);
  return true;
}
/* Callback for FlexCAN_T4 */
static inline void receiveCallback(const CAN_message_t &msg) {
  // Check if the received CAN message exceeds the standard 8-byte length
  if (msg.len > 8) {
    return; // Exit the callback if the message is not supported due to length
  }
  // Pass the received CAN message to a handler function for further processing
  onCanMessage(msg);
}
// Initialize an ODriveCAN object for the first ODrive 0 motor controller
ODriveCAN odrv0(wrap_can_intf(can_0), ODRV0_NODE_ID);
ODriveCAN odrv1(wrap_can_intf(can_0), ODRV1_NODE_ID);
ODriveCAN* odrives[] = {&odrv0, &odrv1};

struct ODriveUserData {
  Heartbeat_msg_t last_heartbeat;
  bool received_heartbeat = false;
  Get_Encoder_Estimates_msg_t last_feedback;
  bool received_feedback = false;
};
ODriveUserData odrv0_user_data;
ODriveUserData odrv1_user_data;

// Callback function to handle heartbeat messages from ODrive
void onHeartbeat(Heartbeat_msg_t& msg, void* user_data) {
  auto* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_heartbeat = msg;
  odrv_user_data->received_heartbeat = true;
}
// Callback function to handle encoder feedback messages from ODrive
void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data) {
  auto* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_feedback = msg;
  odrv_user_data->received_feedback = true;
}
// Function to process incoming CAN messages and route them to the appropriate ODrive object
void onCanMessage(const CAN_message_t& msg) {
  for (auto odrive: odrives) {
    onReceive(msg, *odrive);
  }
}
void stopODrive(ODriveCAN& odrive) {
    odrive.setState(ODriveAxisState::AXIS_STATE_IDLE);
}

/*----------------------------SERIAL FUNCTIONS----------------------------*/
// The parseAxisValue() function is used to parse the axis value from the received string
float parseAxisValue(const String& data) {
  int colonIndex = data.indexOf(':');
  if (colonIndex != -1) {
    String numericPart = data.substring(colonIndex + 1);
    numericPart.trim();
    return numericPart.toFloat();
  }
  return 0; // Return 0 if parsing fails
}

/*----------------------------MAIN SETUP----------------------------*/
void setup() {
  // Serial initialization
  Serial.begin(115200); // Serial Monitor
  Serial2.begin(115200); // Serial - to - USB Communication
  roboclawSerial.begin(115200); // Serial communication with Roboclaw (2nd Axis)
  ticSerial.begin(115200); // Serial communication with Tic (Gripper)
  tic.exitSafeStart(); // Avoid safety features 
  
  // Digital Initialization
  pinMode(DIR, OUTPUT);
  pinMode(PUL, OUTPUT);
  digitalWrite(DIR, LOW);
  digitalWrite(PUL, LOW);

  // Wait for up to 3 seconds for the serial port to be opened on the PC side.
  // If no PC connects, continue anyway.
  for (int i = 0; i < 30 && !Serial; ++i) {
    delay(100);
  }
  delay(200);

  // Register callbacks for the heartbeat and encoder feedback messages
  odrv0.onFeedback(onFeedback, &odrv0_user_data);
  odrv0.onStatus(onHeartbeat, &odrv0_user_data);
  odrv1.onFeedback(onFeedback, &odrv1_user_data);
  odrv1.onStatus(onHeartbeat, &odrv1_user_data);

  // Configure and initialize the CAN bus interface. This function depends on
  // your hardware and the CAN stack that you're using.
  if (!setupCan()) {
    Serial2.println("CAN failed to initialize: reset required");
    while (true); // spin indefinitely
  }

  Serial2.println("Waiting for ODrive...");
  while (!odrv0_user_data.received_heartbeat) {
    pumpEvents(can_0);
    delay(100);
  }

  Serial2.println("found ODrive 0");

  while (!odrv1_user_data.received_heartbeat) {
    pumpEvents(can_0);
    delay(100);
  }

  Serial2.println("found ODrive 1");

  // request bus voltage and current (1sec timeout)
 Serial2.println("attempting to read bus voltage of odrive0 and current");
  Get_Bus_Voltage_Current_msg_t vbus0;
  if (!odrv0.request(vbus0, 1)) {
    Serial2.println("vbus0 request failed!");
    while (true); // spin indefinitely
  }

  Serial2.print("DC voltage [V] of Odrive 0: ");
  Serial2.println(vbus0.Bus_Voltage);
  Serial2.print("DC current [A] of Odrive 0: ");
  Serial2.println(vbus0.Bus_Current);

  while (odrv0_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
    odrv0.clearErrors();
    delay(1);
    // odrv0.setState(ODriveAxisState::AXIS_STATE_ENCODER_OFFSET_CALIBRATION);
    // delay(10000);
    // odrv0.setState(ODriveAxisState::AXIS_STATE_ENCODER_INDEX_SEARCH);
    // delay(1000);    
    odrv0.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    // Pump events for 150ms. This delay is needed for two reasons;
    // 1. If there is an error condition, such as missing DC power, the ODrive might
    //    briefly attempt to enter CLOSED_LOOP_CONTROL state, so we can't rely
    //    on the first heartbeat response, so we want to receive at least two
    //    heartbeats (100ms default interval).
    // 2. If the bus is congested, the setState command won't get through
    //    immediately but can be delayed.
    for (int i = 0; i < 15; ++i) {
      delay(10);
      pumpEvents(can_0);
    }
  }
  Serial2.println("ODrive 0 running!");
  // while (odrv1_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
  //   odrv1.clearErrors();
  //   delay(1);
  //   odrv1.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
  //   // Pump events for 150ms. This delay is needed for two reasons;
  //   // 1. If there is an error condition, such as missing DC power, the ODrive might
  //   //    briefly attempt to enter CLOSED_LOOP_CONTROL state, so we can't rely
  //   //    on the first heartbeat response, so we want to receive at least two
  //   //    heartbeats (100ms default interval).
  //   // 2. If the bus is congested, the setState command won't get through
  //   //    immediately but can be delayed.
  //   for (int i = 0; i < 15; ++i) {
  //     delay(10);
  //     pumpEvents(can_0);
  //   }
  // }
  // Serial2.println("ODrive 1 Finished Calibrating");
  // Serial2.println("ODrive 1 running!");
}

/*----------------------------MAIN LOOP----------------------------*/
void loop() {
    static String receivedData = ""; // Accumulator for received characters
    static float lastOdrive0Value = 0; // Store the last Axis 1 value for debounce logic
    static float lastOdrive1Value = 0; // Store the last Axis 4 value for debounce logic
    static float lastTicAxisValue = 0; // Store the last Gripper value for debounce logic
    static float lastBLDCAxisValue = 0; // Store the last Axis 3 value for debounce logic
    static float lastRoboClawAxisValue = 0; // Store the last Axis 2 value for debounce logic
    
    // Start Data transmitting from ESP32 for control
    while (Serial.available()) {
      char c = Serial.read();
      digitalWrite(LED_BUILTIN, HIGH); // Indicate data reception

      if (c == '\n') { // End of a message
        Serial2.print("Received data: "); //DEBUGGING
        Serial2.println(receivedData); //DEBUGGING

        // /* -------------------- 1st Axis ---------------------- */
        // if (receivedData.startsWith("Axis 3:")) { // Check if the data is for Axis 1
        //   float axisValue1 = parseAxisValue(receivedData);
        //   // Implementing deadzone logic
        //   if (abs(axisValue1) < 0.2) {
        //       axisValue1 = 0; // Treat small values as zero
        //   }

        //   // VELOCITY ACTUATION
        //   if (!isnan(axisValue1) && abs(axisValue1 - lastOdrive0Value) > 0.01) { // Adjust threshold as needed
        //     float odrvVelocity = map(axisValue1 * 1000, -1000, 1000, -ODRIVE_VEL, ODRIVE_VEL);
        //     odrv0.setVelocity(odrvVelocity);
        //     Serial2.print("Setting Odrive 0 velocity to: ");
        //     Serial2.println(odrvVelocity);
        //     lastOdrive0Value = axisValue1; // Update the last known axis value
        //     }
        // }


        /* -------------------- 2nd Axis ---------------------- */
        if (receivedData.startsWith("Axis 2:")) { // Check if the data is for Axis 2 (RoboClaw)
          float axisValue2 = parseAxisValue(receivedData);

          // Implementing deadzone logic
          if (abs(axisValue2) < 0.5) {
            axisValue2 = 0; // Treat small values as zero
          }

          // VELOCITY ACTUATION
          if (!isnan(axisValue2) && abs(axisValue2 - lastRoboClawAxisValue) > 0.01) {
            // Map the absolute value of axisValue2 from 0 to 1 to PWM values from 65 to 127 for forward, and 63 to 0 for backward
            uint8_t pwmValue;
            if (axisValue2 > 0) {
              pwmValue = map(axisValue2 * 1000, 500, 1000, 65, 127); // Forward PWM mapping
            } else {
              pwmValue = map(abs(axisValue2) * 1000, 500, 1000, 63, 1); // Backward PWM mapping
            }

            uint8_t commandByte;
            Serial2.print("Mapped PWM value for RoboClaw: ");
            Serial2.println(pwmValue);

            // Send RoboClaw address before the command and PWM
            roboclawSerial.write(ROBOCLAW_ADDRESS);

            if (axisValue2 > 0) {
              commandByte = 1; // Command byte for M2 Forward
              Serial2.println("Sending Forward command to RoboClaw");
            } else if (axisValue2 < 0) {
              commandByte = 2; // Assuming 2 is the command byte for M2 Backward, as uint8_t cannot be negative
              Serial2.println("Sending Backward command to RoboClaw");
            } else {
              commandByte = 1; // Use Forward command for stopping as well
              pwmValue = 64; // PWM value for stop
              Serial2.println("RoboClaw Park");
            }

            // Send the command byte and PWM value
            roboclawSerial.write(commandByte);
            roboclawSerial.write(pwmValue);

            lastRoboClawAxisValue = axisValue2; // Update the last known axis value
          }
        }



        /* -------------------- 3rd Axis ---------------------- */
        if (receivedData.startsWith("Axis 1:")) { // Check if the data is for Axis 1
           float axisValue3 = parseAxisValue(receivedData);
           // Implementing deadzone logic
           if (abs(axisValue3) < 0.5) {
               axisValue3 = 0; // Treat small values as zero
           }

          // VELOCITY ACTUATION
          if (!isnan(axisValue3) && abs(axisValue3 - lastBLDCAxisValue) > 0.01) { // Adjust threshold as needed
              // Convert the PWM signal to a speed and direction for the stepper motor
              if (axisValue3 >= 0) {
                  // Rotate clockwise
                  digitalWrite(DIR, HIGH);  // Set direction
                  // int steps_cw = map(axisValue3 * 1000, 0, 1000, 0, 100);  // Map the axis value to a range of steps
                  // for (int i = 0; i < steps_cw; i++) {
                  //     // digitalWrite(PUL, HIGH);
                  //     // delayMicroseconds(2);  // Adjust for speed
                  //     // digitalWrite(PUL, LOW);
                  //     // delayMicroseconds(2);  // Adjust for speed
                  // }

                  int speed = map(axisValue3, 0, 1, 0, 255);
                  analogWrite(PUL, speed);
                  // analogWriteFrequency(uint8_t pin, float frequency)
              } 

              if (axisValue3 < 0){
                  // Rotate counterclockwise
                  digitalWrite(DIR, LOW);  // Set direction
                  // int steps_ccw = map(axisValue3, -1000, 0, 0, 100);  // Map the negative axis value to a range of steps
                  // for (int i = 0; i < steps_ccw; i++) {
                  //     // digitalWrite(PUL, HIGH);
                  //     // delayMicroseconds(2);  // Adjust for speed
                  //     // digitalWrite(PUL, LOW);
                  //     // delayMicroseconds(2);  // Adjust for speed
                      
                  // }
                  int speed = map(axisValue3, -1, 0, 255, 0);
                  analogWrite(PUL, speed);
              }

              lastBLDCAxisValue = axisValue3;  // Update the last axis value for the next comparison
          }
        }


        /* -------------------- 4th Axis ---------------------- */
        if (receivedData.startsWith("Axis 0:")) { // Check if the data is for Axis 1
          float axisValue4 = parseAxisValue(receivedData);
          // Implementing deadzone logic
          if (abs(axisValue4) < 0.2) {
              axisValue4 = 0; // Treat small values as zero
          }

          // VELOCITY ACTUATION
          if (!isnan(axisValue4) && abs(axisValue4 - lastOdrive1Value) > 0.01) { // Adjust threshold as needed
            float odrvVelocity = map(axisValue4 * 1000, -1000, 1000, -ODRIVE_VEL, ODRIVE_VEL);
            odrv1.setVelocity(odrvVelocity);
            Serial2.print("Setting Odrive 1 velocity to: ");
            Serial2.println(odrvVelocity);
            lastOdrive1Value = axisValue4; // Update the last known axis value
            }
        }



        /* -------------------- Gripper ---------------------- */
        if (receivedData.startsWith("Axis 3:")) { // Check if the data is for Axis 1
          float axisValue1 = parseAxisValue(receivedData);
           // Implementing deadzone logic
          if (abs(axisValue1) < 0.5) {
               axisValue1 = 0; // Treat small values as zero
          }

           // VELOCITY ACTUATION
          if (!isnan(axisValue1) && abs(axisValue1 - lastTicAxisValue) > 0.01) { // Adjust threshold as needed
            long velocity = map(axisValue1 * 1000, -1000, 1000, -MAX_VELOCITY, MAX_VELOCITY);
            tic.setTargetVelocity(velocity);
            Serial2.print("Setting Tic velocity to: ");
            Serial2.println(velocity);
             lastTicAxisValue = axisValue1; // Update the last known axis value
            }
           }


             receivedData = ""; // Clear for the next message
             digitalWrite(LED_BUILTIN, LOW); // Turn off the LED after processing
           } else if (c != '\r') { // Ignore carriage return characters
             receivedData += c; // Accumulate the next character
           }
           tic.exitSafeStart();
         }
}
