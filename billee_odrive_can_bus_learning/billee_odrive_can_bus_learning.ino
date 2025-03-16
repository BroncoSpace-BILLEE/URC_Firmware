#include <FlexCAN_T4.h>

// Create an object of the FlexCAN library
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;

//Encder Messages
CAN_message_t EncMsg;


void setup(void) {

  // Setup Serial Connection
  Serial.begin(115200); 
  delay(400);

  //Canbus Setup and configuration
  Can0.begin();
  Can0.setBaudRate(500000);
  Can0.setMaxMB(16);
  Can0.enableFIFO();
  Can0.enableFIFOInterrupt();
  Can0.mailboxStatus();
  Can0.onReceive(canSniff);

  //MB Setup
  Can0.onReceive(MB5, printEnc);
  Can0.enableMBInterrupt(MB5);
  Can0.setMBFilter(REJECT_ALL);
  Can0.setMB(MB5, 0xE9);
  Can0.setMB(MB5, RX,STD);

  //Message Setup
  EncMsg.len = 8;
  EncMsg.mb = 5; // MB5
  

}

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


void printEnc(const CAN_message_t &msg){
  Serial.println("ENCOER RECEIVED");
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

void loop() {
  Can0.events();

  // static uint32_t timeout = millis();
  // if ( millis() - timeout > 200 ) {
  //   CAN_message_t msg;
  //   msg.id = random(0x1,0x7FE);
  //   for ( uint8_t i = 0; i < 8; i++ ) msg.buf[i] = i + 1;
  //   Can0.write(msg);
  //   timeout = millis();
  // }

}
