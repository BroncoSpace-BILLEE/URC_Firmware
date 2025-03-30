//Author: Adrian
//purpose: use the crsf library to transcribe data from elrs reciever module
//Version: 0.1
//creation date: 2/24/25

#include "CRSFforArduino.hpp"

void setup() {
  Serial.begin(115200); 
    while (!Serial)
    {
        ;
    }
  crsf = CRSFforArduino(&Serial2);
  if (!crsf->begin())
  {
      crsf->end();

      delete crsf;
      crsf = nullptr;

      Serial.println("CRSF for Arduino initialisation failed!");
      while (1)
      {
          delay(10);
      }
  }
  rcChannelCount = 16;
  Serial.println("RC Channels Example");
  delay(1000);
  Serial.println("Ready");
  delay(1000);
}

void loop() {
  crsf.update();
}
