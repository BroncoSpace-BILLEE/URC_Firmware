#include <Arduino.h>
#include "ODriveCAN.h"
#include <FlexCAN_T4.h>
#include "ODriveFlexCAN.hpp"
#include <SoftwareSerial.h>
/*------------------------------DEFINING PARAMETERS------------------------------*/

// Definition for 1st Axis (ODESC V4.2 CAN BUS)
#define ODRIVE_VEL 5
#define CAN_BAUDRATE 500000
#define ODRV1_NODE_ID 7
#define ODRV2_NODE_ID 2
#define ODRV3_NODE_ID 3
#define ODRV4_NODE_ID 4
#define ODRV5_NODE_ID 5
#define ODRV6_NODE_ID 6

/* Teensy with built-in CAN interface */
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_0;

/*------------------------------ODRIVE (ODESC V4.2) SETUP AND FUNCTIONS------------------------------*/
// Setup Step
struct ODriveStatus; // Hack to prevent Teensy compile error
unsigned long startTime = millis();
bool setupCan() {
  // CAN 0 Setup
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
ODriveCAN odrv1(wrap_can_intf(can_0), ODRV1_NODE_ID);
ODriveCAN odrv2(wrap_can_intf(can_0), ODRV2_NODE_ID);
ODriveCAN odrv3(wrap_can_intf(can_0), ODRV3_NODE_ID);
ODriveCAN odrv4(wrap_can_intf(can_0), ODRV4_NODE_ID);
ODriveCAN odrv5(wrap_can_intf(can_0), ODRV5_NODE_ID);
ODriveCAN odrv6(wrap_can_intf(can_0), ODRV6_NODE_ID);
ODriveCAN* odrives[] = {&odrv2, &odrv3, &odrv4, &odrv5, &odrv1, &odrv6};

struct ODriveUserData {
  Heartbeat_msg_t last_heartbeat;
  bool received_heartbeat = false;
  Get_Encoder_Estimates_msg_t last_feedback;
  bool received_feedback = false;
};

ODriveUserData odrv1_user_data;
ODriveUserData odrv2_user_data;
ODriveUserData odrv3_user_data;
ODriveUserData odrv4_user_data;
ODriveUserData odrv5_user_data;
ODriveUserData odrv6_user_data;

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

/*----------------------------MAIN SETUP----------------------------*/
void setup() {
  // Wait for up to 3 seconds for the serial port to be opened on the PC side.
  // If no PC connects, continue anyway.
  for (int i = 0; i < 30 && !Serial; ++i) {
    delay(100);
  }
  delay(200);

  // Register callbacks for the heartbeat and encoder feedback messages
  odrv1.onFeedback(onFeedback, &odrv1_user_data);
  odrv1.onStatus(onHeartbeat, &odrv1_user_data);
  odrv2.onFeedback(onFeedback, &odrv2_user_data);
  odrv2.onStatus(onHeartbeat, &odrv2_user_data);
  odrv3.onFeedback(onFeedback, &odrv3_user_data);
  odrv3.onStatus(onHeartbeat, &odrv3_user_data);
  odrv4.onFeedback(onFeedback, &odrv4_user_data);
  odrv4.onStatus(onHeartbeat, &odrv4_user_data);
  odrv5.onFeedback(onFeedback, &odrv5_user_data);
  odrv5.onStatus(onHeartbeat, &odrv5_user_data);
  odrv6.onFeedback(onFeedback, &odrv6_user_data);
  odrv6.onStatus(onHeartbeat, &odrv6_user_data);

  // Configure and initialize the CAN bus interface. This function depends on
  // your hardware and the CAN stack that you're using.
  if (!setupCan()) {
    Serial.println("CAN failed to initialize: reset required");
    while (true); // spin indefinitely
  }

  Serial.println("Waiting for ODrive 1...");
  while (!odrv1_user_data.received_heartbeat) {
    pumpEvents(can_0);
    delay(100);
  }
  Serial.println("found ODrive 1");

  Serial.println("Waiting for ODrive 2..");
  while (!odrv2_user_data.received_heartbeat) {
    pumpEvents(can_0);
    delay(100);
  }
  Serial.println("found ODrive 2");
  
  Serial.println("Waiting for ODrive 3...");
  while (!odrv3_user_data.received_heartbeat) {
    pumpEvents(can_0);
    delay(100);
  }
  Serial.println("found ODrive 3");
  
  Serial.println("Waiting for ODrive 4...");
  while (!odrv4_user_data.received_heartbeat) {
    pumpEvents(can_0);
    delay(100);
  }
  Serial.println("found ODrive 4");
  
  Serial.println("Waiting for ODrive 5...");
  while (!odrv5_user_data.received_heartbeat) {
    pumpEvents(can_0);
    delay(100);
  }
  Serial.println("found ODrive 5");
  
  Serial.println("Waiting for ODrive 6...");
  while (!odrv6_user_data.received_heartbeat) {
    pumpEvents(can_0);
    delay(100);
  }
  Serial.println("found ODrive 6");
  

  // request bus voltage and current (1sec timeout)
  Serial.println("attempting to read bus voltage and current");
  Get_Bus_Voltage_Current_msg_t vbus0;
  // odrv2.request(vbus0, 1);
  // odrv3.request(vbus0, 1);
  // odrv4.request(vbus0, 1);
  // odrv5.request(vbus0, 1);
  odrv6.request(vbus0, 1);
  // odrv1.request(vbus0, 1);

  Serial.print("DC voltage [V] of Odrive 0: ");
  Serial.println(vbus0.Bus_Voltage);
  Serial.print("DC current [A] of Odrive 0: ");
  Serial.println(vbus0.Bus_Current);

  Serial.print("Set Axis State: CLOSED LOOP CONTROL");
  while (odrv1_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL ){
    odrv2.clearErrors();
    odrv3.clearErrors();
    odrv4.clearErrors();
    odrv5.clearErrors();
    odrv6.clearErrors();
    odrv1.clearErrors();
    
    delay(500);
    // Serial.println("error Clear");
    odrv2.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    odrv3.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    odrv4.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    odrv5.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    odrv6.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    odrv1.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    delay(500);


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
    // Serial.println("state set");
  }


  Serial.println("ODrives are running!");
}

/*----------------------------MAIN LOOP----------------------------*/
float odrvVelocity = 0;

void loop() {
    // Prompt the user for input
    Serial.println("Enter a new velocity (0-255): ");
    
    // Wait for user input
    while (Serial.available() == 0) {
        // Wait for input
    }
    
    // Read the user input
    odrvVelocity = Serial.parseInt();
    
    // Set the velocity for all motors
    odrv1.setVelocity(odrvVelocity);
    odrv2.setVelocity(odrvVelocity);
    odrv3.setVelocity(odrvVelocity);
    odrv4.setVelocity(odrvVelocity);
    odrv5.setVelocity(odrvVelocity);
    odrv6.setVelocity(odrvVelocity);
    
    // Print the updated velocity
    Serial.print("New velocity set: ");
    Serial.println(odrvVelocity);
    
    // Add a delay to prevent rapid input
    delay(5000);
}
