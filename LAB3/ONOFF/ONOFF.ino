
#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"

// For the Adafruit shield, these are the default.
#define TFT_DC 9
#define TFT_CS 10

// Use hardware SPI (on Uno, #13, #12, #11) and the above for CS/DC
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);
// If using the breakout, change pins as desired
//Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_CLK, TFT_RST, TFT_MISO);


// clearfy the lastState and change for the state out of loop to avoid the flash of "ON" and "OFF"
int lastState = LOW;
int change = 0;

void setup() {
  //start serial connection
  Serial.begin(38400); 

  //pinMode(13, OUTPUT);
  tft.begin();
  tft.setRotation(1);  pinMode(2, INPUT_PULLUP);

  // fill the screen during the settig up state
  tft.fillScreen(ILI9341_BLACK);
  tft.begin();
 // int change = 0;
 // int lastState = LOW;
  
  
}
  


// void loop 
// drawing the ON and OFF once the button is pressure for a long period of time, in 
// this situation, the lastState is equal to sensor value, the image on the LCD stop 
// there

void loop(void) {
    int sensorVal = digitalRead(2);
    Serial.println(sensorVal);
    if(sensorVal != lastState){
      change = 1;
    }else{
      change = 0;
    }
    tft.setRotation(1);
    if (sensorVal == HIGH && change == 1){ 
      digitalWrite(13, LOW);
      tft.fillScreen(ILI9341_BLACK);
      tft.setTextColor(ILI9341_YELLOW);
      tft.setTextSize(5);
      tft.setCursor(125, 100);
      tft.print("OFF");
    } else if(sensorVal == LOW && change == 1){
      digitalWrite(13, HIGH);
      tft.fillScreen(ILI9341_BLACK);
      tft.setTextColor(ILI9341_YELLOW);
      tft.setTextSize(5);
      tft.setCursor(125, 100);
      tft.print("ON");
    }
    lastState = sensorVal;
}





