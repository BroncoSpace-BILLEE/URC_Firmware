

void colorBitHandler(){
    switch (color_bit) {

    case 1:
      // Serial.println(color_bit);
      breatheRed();
      break;
    case 2:
      // Serial.println(color_bit);
      breatheGreen();
      break;
    case 3:
      // Serial.println(color_bit);
      breatheBlue();
      break;
    default:
      NeoPixel.clear();
      break;
    }

}



void flashRed() {
  NeoPixel.clear();
  for (int i = 0; i <= 10; i++) {
    for (int pixel = 0; pixel < NUM_PIXELS; pixel++) {           // for each pixel
      NeoPixel.setPixelColor(pixel, NeoPixel.Color(255, 0, 0));  // it only takes effect if pixels.show() is called
    }
    NeoPixel.show();
    delay(500);
    NeoPixel.clear();
    NeoPixel.show();
    delay(500); 
  }
}

void flashGreen() {
  NeoPixel.clear();
  for (int i = 0; i <= 10; i++) {
    for (int pixel = 0; pixel < NUM_PIXELS; pixel++) {           // for each pixel
      NeoPixel.setPixelColor(pixel, NeoPixel.Color(0, 255, 0));  // it only takes effect if pixels.show() is called
    }
    NeoPixel.show();
    delay(500);
    NeoPixel.clear();
    NeoPixel.show();
    delay(500); 
  }
}

void flashBlue() {
  NeoPixel.clear();
  for (int i = 0; i <= 10; i++) {
    for (int pixel = 0; pixel < NUM_PIXELS; pixel++) {           // for each pixel
      NeoPixel.setPixelColor(pixel, NeoPixel.Color(0, 0, 255));  // it only takes effect if pixels.show() is called
    }
    NeoPixel.show();
    delay(500);
    NeoPixel.clear();
    NeoPixel.show();
    delay(500); 
  }
}

void breatheRed() {
  NeoPixel.clear();
  for (int i = 0; i < NUM_REPEAT; i++) {
    for (int i = 0; i <= 255; i+=2) {
      for (int pixel = 0; pixel < NUM_PIXELS; pixel++)      // for each pixel
        NeoPixel.setPixelColor(pixel, NeoPixel.Color(i, 0, 0));
      NeoPixel.show();
      delay(2);
    }
    for (int i = 255; i >= 0; i-=2) {
      for (int pixel = 0; pixel < NUM_PIXELS; pixel++)      // for each pixel
        NeoPixel.setPixelColor(pixel, NeoPixel.Color(i, 0, 0));
      NeoPixel.show();
      delay(2);
    }
  }
}

void breatheGreen() {
  NeoPixel.clear();
  for (int i = 0; i < NUM_REPEAT; i++) {
    for (int i = 0; i <= 255; i+=2) {
      for (int pixel = 0; pixel < NUM_PIXELS; pixel++)      // for each pixel
        NeoPixel.setPixelColor(pixel, NeoPixel.Color(0, i, 0));
      NeoPixel.show();
      delay(2);
    }
    for (int i = 255; i >= 0; i-=2) {
      for (int pixel = 0; pixel < NUM_PIXELS; pixel++)      // for each pixel
        NeoPixel.setPixelColor(pixel, NeoPixel.Color(0, i, 0));
      NeoPixel.show();
      delay(2);
    }
  }
}

void breatheBlue() {
  NeoPixel.clear();
  for (int i = 0; i < NUM_REPEAT; i++) {
    for (int i = 0; i <= 255; i+=2) {
      for (int pixel = 0; pixel < NUM_PIXELS; pixel++)      // for each pixel
        NeoPixel.setPixelColor(pixel, NeoPixel.Color(0, 0, i));
      NeoPixel.show();
      delay(2);
    }
    for (int i = 255; i >= 0; i-=2) {
      for (int pixel = 0; pixel < NUM_PIXELS; pixel++)      // for each pixel
        NeoPixel.setPixelColor(pixel, NeoPixel.Color(0, 0, i));
      NeoPixel.show();
      delay(2);
    }
  }
}