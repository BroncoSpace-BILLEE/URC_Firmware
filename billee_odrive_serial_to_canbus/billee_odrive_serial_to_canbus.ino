#include <Arduino.h>
#include "ODriveCAN.h"
#include <FlexCAN_T4.h>
#include "ODriveFlexCAN.hpp"
#include <SoftwareSerial.h>
/*------------------------------DEFINING PARAMETERS------------------------------*/

//Teensy 4.1 LED
#define TEENSY_LED 13 //built-in LED

// TIME TO CALIBRATE - ODRIVES
#define ODRIVE_CALIBRATION_TIME 25 //seconds

//Definition for 1st Axis (ODESC V4.2 CAN BUS)
#define ODRIVE_TIME_TO_CALIBRATE 15 // Seconds to wait until offset calibration done
#define ODRIVE_NODE_ID_SHIFT 5
#define CAN_BAUDRATE 500000
#define ODRV1_NODE_ID 7
#define ODRV2_NODE_ID 2
#define ODRV3_NODE_ID 3
#define ODRV4_NODE_ID 4
#define ODRV5_NODE_ID 5
#define ODRV6_NODE_ID 6

//Definition of Encoder CMD_ID
#define ODRIVE_ENC_CMD_ID 0x09 // Defined by Odrive
#define ODRV1_ENC_CMD_ID (ODRV1_NODE_ID<<ODRIVE_NODE_ID_SHIFT)|ODRIVE_ENC_CMD_ID
#define ODRV2_ENC_CMD_ID (ODRV2_NODE_ID<<ODRIVE_NODE_ID_SHIFT)|ODRIVE_ENC_CMD_ID
#define ODRV3_ENC_CMD_ID (ODRV3_NODE_ID<<ODRIVE_NODE_ID_SHIFT)|ODRIVE_ENC_CMD_ID
#define ODRV4_ENC_CMD_ID (ODRV4_NODE_ID<<ODRIVE_NODE_ID_SHIFT)|ODRIVE_ENC_CMD_ID
#define ODRV5_ENC_CMD_ID (ODRV5_NODE_ID<<ODRIVE_NODE_ID_SHIFT)|ODRIVE_ENC_CMD_ID
#define ODRV6_ENC_CMD_ID (ODRV6_NODE_ID<<ODRIVE_NODE_ID_SHIFT)|ODRIVE_ENC_CMD_ID

//Definition of heartbeat ID
#define ODRIVE_HEARTBEAT_CMD_ID 0x01 // Defined by Odrive
#define ODRV1_HB_CMD_ID (ODRV1_NODE_ID<<ODRIVE_NODE_ID_SHIFT)|ODRIVE_HEARTBEAT_CMD_ID
#define ODRV2_HB_CMD_ID (ODRV2_NODE_ID<<ODRIVE_NODE_ID_SHIFT)|ODRIVE_HEARTBEAT_CMD_ID
#define ODRV3_HB_CMD_ID (ODRV3_NODE_ID<<ODRIVE_NODE_ID_SHIFT)|ODRIVE_HEARTBEAT_CMD_ID
#define ODRV4_HB_CMD_ID (ODRV4_NODE_ID<<ODRIVE_NODE_ID_SHIFT)|ODRIVE_HEARTBEAT_CMD_ID
#define ODRV5_HB_CMD_ID (ODRV5_NODE_ID<<ODRIVE_NODE_ID_SHIFT)|ODRIVE_HEARTBEAT_CMD_ID
#define ODRV6_HB_CMD_ID (ODRV6_NODE_ID<<ODRIVE_NODE_ID_SHIFT)|ODRIVE_HEARTBEAT_CMD_ID

/* Teensy with built-in CAN interface */
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_0;

/*----------------------- Odrive Can definition---------------------*/

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

/*-----------------------------CAN Encoder Definitions -------------------*/

float node1Vel = 0;
float node2Vel = 0;
float node3Vel = 0;
float node4Vel = 0;
float node5Vel = 0;
float node6Vel = 0;

float node1EncCount = 0;
float node2EncCount = 0;
float node3EncCount = 0;
float node4EncCount = 0;
float node5EncCount = 0;
float node6EncCount = 0;

String serialInput = "0,0,0,0,0,0";

Get_Encoder_Estimates_msg_t Enc1;
Get_Encoder_Estimates_msg_t Enc2;
Get_Encoder_Estimates_msg_t Enc3;
Get_Encoder_Estimates_msg_t Enc4;
Get_Encoder_Estimates_msg_t Enc5;
Get_Encoder_Estimates_msg_t Enc6;

CAN_message_t EncRequestMsg; // For sending Encoder Requests


/*------------------------------ODRIVE (ODESC V4.2) SETUP AND FUNCTIONS------------------------------*/
// Setup Step
struct ODriveStatus; // Hack to prevent Teensy compile error
unsigned long startTime = millis();

/*  Initial CAN Setup for Odrive Initialization   */
bool setupCan() {

  // CAN 0 Setup
  can_0.begin();
  can_0.setBaudRate(CAN_BAUDRATE);
  can_0.setMaxMB(17);
  can_0.enableFIFO();
  can_0.enableFIFOInterrupt();
  can_0.onReceive(receiveCallback);

  return true;
}

/*  Setup Flexcan_T4 Mailboxes for Encoder requests  */
void SetupCANMailbox(){

  can_0.setMBFilter(REJECT_ALL);

  // Setup MB for receiving standard CAN frames for Encoder messages. 

  can_0.setMB(MB11,RX,STD);
  can_0.setMB(MB12,RX,STD);
  can_0.setMB(MB13,RX,STD);
  can_0.setMB(MB14,RX,STD);
  can_0.setMB(MB15,RX,STD);
  can_0.setMB(MB16,RX,STD);

  can_0.enableMBInterrupts(MB11);
  can_0.enableMBInterrupts(MB12);
  can_0.enableMBInterrupts(MB13);
  can_0.enableMBInterrupts(MB14);
  can_0.enableMBInterrupts(MB15);
  can_0.enableMBInterrupts(MB16);


  can_0.onReceive(MB11,CANEncRequestMsgHandler);
  can_0.onReceive(MB12,CANEncRequestMsgHandler);
  can_0.onReceive(MB13,CANEncRequestMsgHandler);
  can_0.onReceive(MB14,CANEncRequestMsgHandler);
  can_0.onReceive(MB15,CANEncRequestMsgHandler);
  can_0.onReceive(MB16,CANEncRequestMsgHandler);


  can_0.setMBFilter(MB11,ODRV1_ENC_CMD_ID);
  can_0.setMBFilter(MB12,ODRV2_ENC_CMD_ID);
  can_0.setMBFilter(MB13,ODRV3_ENC_CMD_ID);
  can_0.setMBFilter(MB14,ODRV4_ENC_CMD_ID);
  can_0.setMBFilter(MB15,ODRV5_ENC_CMD_ID);
  can_0.setMBFilter(MB16,ODRV6_ENC_CMD_ID);


  can_0.mailboxStatus();

}

/* Request the Encoder Data from Odrives */
void EncoderRequest(){

  //Setup Encder Request message
  EncRequestMsg.flags.remote = 1;
  EncRequestMsg.len = 0;

  //Request Encoder 1
  EncRequestMsg.id = ODRV1_ENC_CMD_ID;
  can_0.write(EncRequestMsg);

  pumpEvents(can_0);

  //Request Encoder 2
  EncRequestMsg.id = ODRV2_ENC_CMD_ID;
  can_0.write(EncRequestMsg);

  pumpEvents(can_0);

  //Request Encoder 3
  EncRequestMsg.id = ODRV3_ENC_CMD_ID;
  can_0.write(EncRequestMsg);

  pumpEvents(can_0);

  //Request Encoder 4
  EncRequestMsg.id = ODRV4_ENC_CMD_ID;
  can_0.write(EncRequestMsg);

  pumpEvents(can_0);

  //Request Encoder 5
  EncRequestMsg.id = ODRV5_ENC_CMD_ID;
  can_0.write(EncRequestMsg);

  pumpEvents(can_0);

  //Request Encoder 6
  EncRequestMsg.id = ODRV6_ENC_CMD_ID;
  can_0.write(EncRequestMsg);

  pumpEvents(can_0);

}

/*  Convert Serial Commands to Odrive CAN commands  */
void OdriveSerialControl(){

    // Wait for user input to update node velocities
    if (Serial.available()) {
        // Read the user input
      serialInput = Serial.readStringUntil('\n');
      Serial.flush();
    }

    //Extract velocity values
    sscanf(serialInput.c_str(),"%f,%f,%f,%f,%f,%f", &node1Vel, &node2Vel, &node3Vel, &node4Vel, &node5Vel, &node6Vel);
    
    // Set the velocity for all motors
    odrv1.setVelocity(node1Vel);
    odrv2.setVelocity(node2Vel);
    odrv3.setVelocity(node3Vel);
    odrv4.setVelocity(node4Vel);
    odrv5.setVelocity(node5Vel);
    odrv6.setVelocity(node6Vel);
    
    // Reset node Velocities to 0
    node1Vel=0;
    node2Vel=0;
    node3Vel=0;
    node4Vel=0;
    node5Vel=0;
    node6Vel=0;
}

void EncoderFeedback(){
  EncoderRequest();

  // print values for encoders
  Serial.print(-1*Enc1.Pos_Estimate);
  Serial.print(",");
  Serial.print(-1*Enc2.Pos_Estimate);
  Serial.print(",");
  Serial.print(-1*Enc3.Pos_Estimate);
  Serial.print(",");
  Serial.print(Enc4.Pos_Estimate);
  Serial.print(",");
  Serial.print(Enc5.Pos_Estimate);
  Serial.print(",");
  Serial.print(Enc6.Pos_Estimate);
  Serial.println();
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

void OdriveSetupCANControl(){

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
    delay(21);
  }
  Serial.println("found ODrive 1");

  Serial.println("Waiting for ODrive 2..");
  while (!odrv2_user_data.received_heartbeat) {
    pumpEvents(can_0);
    delay(23);
  }
  Serial.println("found ODrive 2");
  
  Serial.println("Waiting for ODrive 3...");
  while (!odrv3_user_data.received_heartbeat) {
    pumpEvents(can_0);
    delay(27);
  }
  Serial.println("found ODrive 3");
  
  Serial.println("Waiting for ODrive 4...");
  while (!odrv4_user_data.received_heartbeat) {
    pumpEvents(can_0);
    delay(27);
  }
  Serial.println("found ODrive 4");
  
  Serial.println("Waiting for ODrive 5...");
  while (!odrv5_user_data.received_heartbeat) {
    pumpEvents(can_0);
    delay(27);
  }
  Serial.println("found ODrive 5");
  
  Serial.println("Waiting for ODrive 6...");
  while (!odrv6_user_data.received_heartbeat) {
    pumpEvents(can_0);
    delay(27);
  }
  Serial.println("found ODrive 6");


  Serial.print("Set Axis State: CLOSED LOOP CONTROL");
  while (odrv1_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL ){
    odrv2.clearErrors();
    odrv3.clearErrors();
    odrv4.clearErrors();
    odrv5.clearErrors();
    odrv6.clearErrors();
    odrv1.clearErrors();
    
    delay(100);
    // Serial.println("error Clear");
    odrv2.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    odrv3.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    odrv4.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    odrv5.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    odrv6.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    odrv1.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    delay(100);


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

/*-----------------------------CAN MB Setup------------------*/
void canSniff(const CAN_message_t &msg) {
  Serial.print("MB "); Serial.print(msg.mb);
  Serial.print("  OVERRUN: "); Serial.print(msg.flags.overrun);
  Serial.print("  LEN: "); Serial.print(msg.len);
  Serial.print(" EXT: "); Serial.print(msg.flags.extended);
  Serial.print(" TS: "); Serial.print(msg.timestamp);
  Serial.print(" ID: "); Serial.print(msg.id, HEX);
  Serial.print(" Buffer: ");
  for ( uint8_t i = 0; i < msg.len; i++ ) {
    Serial.print(msg.buf[i], HEX); Serial.print(" ");
  } Serial.println();
}

void CANEncRequestMsgHandler(const CAN_message_t &msg){
  switch(msg.id){
    case ODRV1_ENC_CMD_ID:
      //copy the contents of the msg array 
      Enc1.decode_buf(msg.buf);
      break;

    case ODRV2_ENC_CMD_ID:
      //copy the contents of the msg array 
      Enc2.decode_buf(msg.buf);
      break;

    case ODRV3_ENC_CMD_ID:
      //copy the contents of the msg array 
      Enc3.decode_buf(msg.buf);
      break;

    case ODRV4_ENC_CMD_ID:
      //copy the contents of the msg array 
      Enc4.decode_buf(msg.buf);
      break;

    case ODRV5_ENC_CMD_ID:
      //copy the contents of the msg array 
      Enc5.decode_buf(msg.buf);
      break;

    case ODRV6_ENC_CMD_ID:
      //copy the contents of the msg array 
      Enc6.decode_buf(msg.buf);
      break;
  }
}

/*----------------------------MAIN SETUP----------------------------*/

void setup() {
  // Wait for up to 3 seconds for the serial port to be opened on the PC side.
  // If no PC connects, continue anyway.
  for (int i = 0; i < 30 && !Serial; ++i) {
    delay(10);
  }
  delay(1000);


  // // DElAY until ODRIVE can is finished calibrating
  delay(ODRIVE_CALIBRATION_TIME*1000);

  // Setup ODrive -> Close Loop Velocity Control
  OdriveSetupCANControl();

  // Disable FIFO 
  can_0.disableFIFO();
  can_0.disableFIFOInterrupt();

  //Setup Mailbox 
  SetupCANMailbox();

  //LED
  pinMode(TEENSY_LED, OUTPUT);
  digitalWrite(TEENSY_LED, HIGH);

}


/*----------------------------MAIN LOOP----------------------------*/

void loop() {

  pumpEvents(can_0);

  OdriveSerialControl();

  pumpEvents(can_0);

  EncoderFeedback();

  delay(5);
  
}





