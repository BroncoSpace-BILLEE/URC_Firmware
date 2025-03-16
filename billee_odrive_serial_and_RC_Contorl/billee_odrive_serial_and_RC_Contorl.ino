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

/*---------------------------------------------RC Control Definitions -------------------------------------------------*/
#define CH1 19 // Channel 1 of TGY-i6 Reciever
#define CH2 18 // Channel 2 of TGY-i6 Receiver
#define CH3 17 // Channel 3 of TGY-i6 Receiver
#define CH4 16 // Channel 4 of TGY-i6 Receiver
#define CH5 15 // Channel 5 of TGY-i6 Receiver
#define CH6 14 // Channel 6 of TGY-i6 Receiver

#define MODE_SELECT_PIN_B 20 // Hardware Switch (1) connected to this pin
#define MODE_SELECT_PIN_A 21 // Hardware Switch (2) connected to this pin

int ch1Value = 0;
int ch2Value = 0;
int ch5Value = 0;
bool ch6Value = false;

bool InRCControl = false;
bool InSerialControl = true;

/*------------------------------ODRIVE (ODESC V4.2) SETUP AND FUNCTIONS------------------------------*/
// Setup Step
struct ODriveStatus; // Hack to prevent Teensy compile error
unsigned long startTime = millis();

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

  //Setup Mode select pins
  pinMode(MODE_SELECT_PIN_A, INPUT_PULLUP);
  pinMode(MODE_SELECT_PIN_B, INPUT_PULLUP);

  // Setup for RC 
  RCSetupPins();

  //Check the state of MODE select -> Select Drive Mode
  if( (digitalRead(MODE_SELECT_PIN_A)== LOW) && (digitalRead(MODE_SELECT_PIN_B)==LOW) ){
    InRCControl = true;
    InSerialControl = false;
  } else {
    InSerialControl = true;
    InRCControl = false;
  }

}


/*----------------------------MAIN LOOP----------------------------*/

void loop() {

  while(InSerialControl){
    pumpEvents(can_0);

    OdriveSerialControl();

    pumpEvents(can_0);

    EncoderFeedback();

    delay(5);
  } 

  while(InRCControl){
    RCPollValueScaled();

    RCTankDrive();

    RCPrintDebug();

    delay(101);
  }
  
  
}





