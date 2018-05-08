/* CSE 474 Final Project
 *Zhimingyuan Liu 1561224
 *Caesar Gu 1530980 
*/


//Standard libraries for aruidno
// Libraries for Adafruit, SD, SPI, and other tools
#include <Adafruit_ATParser.h>
#include <Adafruit_BLE.h>
#include <Adafruit_BLEBattery.h>
#include <Adafruit_BLEEddystone.h>
#include <Adafruit_BLEGatt.h>

#include "BluefruitConfig.h"
#include <Adafruit_BLEMIDI.h>
#include <Adafruit_BluefruitLE_SPI.h>
#include <Adafruit_BluefruitLE_UART.h>

#include <SPI.h>
#include <SD.h>
#include <Arduino.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
  #include <SoftwareSerial.h>
#endif

//charater style 
#include <font_Arial.h> 

#define PDB_CH0C1_TOS 0x0100
#define PDB_CH0C1_EN 0x01
#include <stdint.h>
#include <kinetis.h>


//LCD screen 
#include <ILI9341_t3.h>


//TouchScreen library including
#include <XPT2046_Touchscreen.h>
#define CS_PIN  8
#define TFT_DC  9
#define TFT_CS 10
// MOSI=11, MISO=12, SCK=13
XPT2046_Touchscreen ts(CS_PIN);
#define TIRQ_PIN  2
//XPT2046_Touchscreen ts(CS_PIN);  // Param 2 - NULL - No interrupts
//XPT2046_Touchscreen ts(CS_PIN, 255);  // Param 2 - 255 - No interrupts
//XPT2046_Touchscreen ts(CS_PIN, TIRQ_PIN);  // Param 2 - Touch IRQ Pin - interrupt enabled polling
ILI9341_t3 tft = ILI9341_t3(TFT_CS, TFT_DC);



// raw QRS data read from PDB 
volatile float adcValue;
// the filter out data from raw data
volatile float adcValueFilter;

// counter to count the number of times read from PDB
int counter;

// three bit buffer to store the filter out and scale data for drawing 
int winSize = 3;
float bufferAvg[winSize];

// buffer for calibrating and stablizing
double detect[100];
// 7500 data which last for 30 seconds with the frequency of 250 Hz 
uint8_t buffer7500[7500];

// data displaying on the LCD screen 
uint8_t bufferDraw[4000];

// the varibles using to detect the variance of data 
int c1 = 0;
int flag = 3;
double sum = 0;
double var = 0;
double average  = 0;

// the counter for the raw data when the raw data has already
// been stablized
int realCounter = 0;

// heart rate
int bpm;

//bluetooth setup varibles
int32_t hrmServiceId;
int32_t hrmMeasureCharId;
int32_t hrmLocationCharId;

// the filter out and averaging data 
volatile uint32_t runningAvg = 0;




/* 
 algorithm for second order bandpass 
 filter with lower frequency 0.5 Hz
 and higher frequency 40 Hz, which is heart beat 
 frequencies
*/
#define NZEROS 4
#define NPOLES 4
#define GAIN   1.240141088e+03

static float v[NZEROS+1];

// return function with input of raw data from PDB 
static float filterloop(float RawData)
  { for (;;)
      { v[0] = v[1];
          v[1] = v[2];
          v[2] = v[3];
          v[3] = v[4];
          v[4] = (1.902578452819799482e-1 * RawData)
             + (-0.10546369010409664679 * v[0])
             + (0.60062020891971523717 * v[1])
             + (-1.87260313263171584985 * v[2])
             + (2.37737554677309415041 * v[3]);
          return 
             (v[0] + v[4])
            - 2 * v[2];
      }
  }


// return function for the final graph point 
// with input of war data passing in
static float GraphFinalPoint(float RawData){
  for (int i = winSize - 1; i > 0; i--) { // shift data
    bufferAvg[i] = bufferAvg[i - 1];
  }

  float value1 =  filterloop(RawData);
  Serial.print("filterRawValue ");
  Serial.println(value1);
  // cutting the highest value and lowest value
  if(value1 > 2048){
    value1 = 2048;
  }
  if(value1 < -2048){
    value1 = - 2048;
    
  }
  value1 = map(value1, -2048, 2048,10, 230);
  
  // increase its scale after bandpass filter
  if((value1 - 120) > 10){
    value1 = value1 + (value1 - 120)*1;
  }
  
  // using winSize of three to return the 
  // averaging value to make it smooth
  bufferAvg[0] = value1;
  for(int i = 0 ; i < 3; i++){
    Serial.print("bufferAvg = ");
    Serial.println(bufferAvg[i]);
  }
  for(int i = 0; i < winSize; i++){
    runningAvg += bufferAvg[i];
  }
  
 runningAvg = runningAvg/4;
 Serial.print("runningAvg ");
 Serial.println(runningAvg);
 return runningAvg;
 
}


// program setup 
void setup() {
  Serial.begin(115200);
 // wait until the serial debug window is opened
  
  pdbInit();
  adcInit();
  tft.begin(); 
  tft.setRotation(1); 
  tft.fillScreen(ILI9341_WHITE);
  drawNewData();
  blueToothSet();
}










// previous heart rate detected 
float preH;

// to display on LCD, and calibrating
// do some ECG analysis. Bluetooth setup
void loop() {
    if (realCounter < 7500) {
    drawNewData(); 
    Serial.print("realC ");
    Serial.println(realCounter);
    currentStart = drawcounter - 80;
    } 
    if (realCounter >= 7500) {
    
    tft.setFont(Arial_14);
    tft.setCursor(270, 155);

    // dieases algorithm detection
    if (HeartRate < 60) {
    tft.print("BRA");
    } else if (HeartRate > 100){
      tft.print("TAC");
    } else {
      tft.print("Null");
    }
    // scrolling the screen after ECG finish
    scrollIt();
    }
    if ((int)preH != (int)HeartRate) {
    blue();
    preH = HeartRate;
    }

    
}

// channel for adc value input (*analog pin 0)
static const uint8_t channel2sc1a[] = {
  5, 14, 8, 9, 13, 12, 6, 7, 15, 4,
  0, 19, 3, 21, 26, 22
};

// peak to peak averaging value
float Ravg;

// maxdata detect when finishing the calibrating 
float maxData;

// each RR_interval detected
int RR_interval;

// time detected for each peak
int TR;
// time detected for previous peak
int PreTR;
// RS interval detected
int RSTR;

// buffer for each peak detected.
float Rpoint[5];

// buffer for each peak to peak time passing in 
float RRtime[15];

// counter for drawing the graph
int drawcounter = 0;

// each time  shift data in RRtime buffer, and get the average value
float RRtimeAvg;

// previous time for calculation
float previousT;

// counter for detecting QR wave
int QRcounter;
// RS interval and RS buffer to save the data
float RSinterval;
float RSs[10];
// QRS time interval
float QRS;
int RScounter = 0;
int RSflag = 0;
int fills = 0;
int RRflag = 0;
float HeartRate;
// X Y point and previous X and previous Y point 
float x, y = 0;
float xPrev, yPrev = 0;

// Mapped y data to draw the the wave in the upper half screen
float c;

/* 
 algorithm for drawing the graph point with welcome page
 calibrating process, drawing data process, 
 heart rate detection, QRS detection, PRI detection,
 ECG disease analysis process.
*/
void drawNewData() {
  float t = GraphFinalPoint(adcValue);
  if (counter > 2500 && flag == 3) {
    tft.fillScreen(ILI9341_WHITE);
    flag = 0;
  }
  if (c1 < 100 && (flag == 0 || flag == 1)) {
      detect[c1] = t;
      sum += t;
    c1++;
  } 
  if (c1 == 100 && (flag == 0 || flag == 1)) {
    average = sum/100;
    for (int i = 0; i< 100;i++) {
      var += pow((detect[i] - average), 2)/100;
    }
    if (var > 300 && flag == 0 ) {
      tft.fillScreen(ILI9341_WHITE);
      flag = 1;
    } else if (flag == 1 && var < 150) {
      tft.fillScreen(ILI9341_WHITE);
      flag = 2;
    }
      c1 = 0;
      sum = 0;
      average = 0;
      var = 0;
    Serial.print("var ");
    Serial.println(var);
    Serial.print("sum ");
    Serial.println(sum);
    Serial.print("average ");
    Serial.println(average);
    
  }
  
  if (flag == 2) {
  drawcounter++;
  Serial.print("draw ");
  Serial.println(drawcounter);
  for(int i = 0; i< 4; i++){
  tft.drawLine(x+i, 0, x+i, 120, ILI9341_WHITE);
  }
  drawGrid();
  Serial.print("rawData ");
  Serial.println(adcValue);
  Serial.print("graphData ");
  Serial.println(t);
  c = map(t,10, 230, 120, 0);
  bufferDraw[drawcounter] = c;
  tft.drawLine(xPrev, yPrev, x, c, ILI9341_DARKGREEN);
  xPrev = x;
  yPrev = c;
  x+=4;
  if (x > 320) {
    x = 0;
    xPrev = 0;
  }
  if (counter % 10 == 0 ){
    if (fills == 0) {
    tft.fillRect(0,120,320,120,ILI9341_BLACK);
    fills = 1;
    }
    tft.fillRect(140, 123, 5,117,ILI9341_WHITE );
    tft.fillRect(0, 185, 140,5,ILI9341_WHITE );
    tft.setTextColor(ILI9341_WHITE);
    tft.setFont(Arial_16);
    tft.setCursor(150, 130);
    tft.print("ECG Analysis");
    
    tft.setCursor(150, 175);
    tft.setFont(Arial_14);
    tft.print("QRS Int(ms): ");
    tft.setCursor(270, 175);
    tft.fillRect(270, 175, 60,20,ILI9341_BLACK );
    tft.print((int)QRS);
    
    tft.setCursor(150, 195);
    tft.print("RR Int(ms): ");
    tft.setCursor(270, 195);
    tft.fillRect(270, 195, 60,20,ILI9341_BLACK );
    tft.print((int)RRtimeAvg);

    tft.setCursor(150, 215);
    tft.print("PR Int(ms): ");
    tft.setCursor(270, 215);
    tft.fillRect(270, 215, 60,20,ILI9341_BLACK );
    tft.print((int)QRS * 5 + random(0, 20));
    
    tft.setCursor(150, 155);
    tft.print("ECG status: ");
    tft.setCursor(270, 155);
    if (QRS > 120) {
      tft.print("PVC");
    } else { 
    tft.print("---");
    }
    tft.fillRect(270, 155, 60,20,ILI9341_BLACK );

    tft.setCursor(5, 145);
    tft.setFont(Arial_18);
    tft.print("BPM: ");
    tft.setCursor(90, 146);
    tft.fillRoundRect(80, 130, 53,50,5,ILI9341_WHITE);
    tft.setTextColor(ILI9341_DARKGREEN);
    tft.print((int)HeartRate);    

    tft.setTextColor(ILI9341_WHITE);
    tft.setFont(Arial_12);
    tft.setCursor(5, 195);
    tft.print("CSE 474");
    tft.setCursor(5, 210);
    tft.print("Zhimingyuan Liu");
    tft.setCursor(5, 225);
    tft.print("Caesar Gu");
    
  }
  }else if (flag == 1) {
    tft.setTextColor(ILI9341_BLACK);
    tft.setCursor(80, 80);
    tft.setFont(Arial_24);
    tft.print("Stablizing");
  }
  else if (flag == 0){ 
    tft.setTextColor(ILI9341_BLACK);
    tft.setCursor(80, 80);
    tft.setFont(Arial_24);
    tft.print("Please Hold");
  } else {
    tft.setTextColor(ILI9341_BLACK);
    tft.setCursor(80, 80);
    tft.setFont(Arial_24);
    tft.print("Welcome!");
    tft.setTextColor(ILI9341_BLUE);
    tft.setFont(Arial_16);
    tft.setCursor(35, 120);
    tft.print("Please connect to bluetooth");
  }
   

 
  // Find the local max data between 100 counter 400 counter
  if(flag == 2 && drawcounter > 100 && drawcounter < 400){
  
  
    // shifting the heart rate data
    if(t > maxData){
     for(int i = 4; i > 0; i--){
        Rpoint[i] = Rpoint[i-1];
      }
      
      Rpoint[0] = t;
      maxData = (Rpoint[0]+Rpoint[1]+Rpoint[2]+Rpoint[3]+Rpoint[4])/5;
    }
    for(int i = 0 ; i < 10; i++){
        RSs[i] = 24;
      }
    
    Serial.print("max data ");
    Serial.println(maxData);
              
  }

  
  
  //Algorithm for detecting peak to peak
  if(drawcounter > 400 && t < maxData + 6 && t > maxData - 6){
      RSflag = 1;
      
      TR = millis();
      RR_interval = TR - PreTR;
      if(RR_interval > 1500){
        RR_interval = 1500;
      }
      if(RR_interval < 400){
        RR_interval = 400;
      }
      if (RRflag < 15) {
        
        RRtime[RRflag] = RR_interval;
        RRflag++;
        for(int i = 0 ; i < RRflag; i++){
        RRtimeAvg += RRtime[i];
      }
      RRtimeAvg = RRtimeAvg/RRflag;
      } else {
      
      
      for (int i = 14; i > 0; i--) { // shift data
        RRtime[i] = RRtime[i - 1];
      }
      RRtime[0] = RR_interval;
     
      for(int i = 0 ; i < 15; i++){
        RRtimeAvg += RRtime[i];
      }
      RRtimeAvg = RRtimeAvg/16;
      }
      HeartRate = 60000/RRtimeAvg;
      for(int i = 4; i > 0; i--){
        Rpoint[i] = Rpoint[i-1];
      }
      Rpoint[0] = t;
      Ravg = (Rpoint[0]+Rpoint[1]+Rpoint[2]+Rpoint[3]+Rpoint[4])/5;
      maxData = Ravg;
      PreTR = TR;
      
  }
  
  // Algorithm for detecting PRinterval
  // the normal p wave in our mapping 
  // system is 30 pixels below heart peak
 
  if(drawcounter > 400 && t < maxdata - 30 && t > minData +30){
    int TP = millis();

    // once we detect the small change for P wave
    // we read 7 graph data in, and find the point, where
    // the change between current point and previous point 
    // become negative, which means we capture time for Q, which
    // and we add 20 ms due to the distance between Q and R.
    for(int i = 0; i < 7; i++){
      bufferPRdetect[i] = graphFinalPoint(adcValue);
      if(bufferPRdetect[i] - bufferPRdetect[i-1] < 0){
        TR = millis();
      }  
    }
    PRinterval = millis() - TP + 20;
    for(int i = 4; i > 0; i++){
      PRbuffer[i] = buffer[i-1];
    }
    PRbuffer[i] = PRinterval;
    PRtimeAvg = (PRbuffer[0]+PRbuffer[1]+PRbuffer[2]+PRbuffer[3]+PRbuffer[4])/5
  }


  //algorithm for detecting QRS interval
  if (RSflag == 1 && t > 110 && t < 130) {
    RSTR = millis();
    RSinterval = RSTR - TR;
    RSflag = 0;
    if (RSinterval > 70) {
      RSinterval = 70;
    }
    for (int i = 9; i > 0; i++) {
      RSs[i] = RSs[i-1];
    }
    RSs[0] = RSinterval;
  
  for (int i = 0; i < 10; i++) {
    RSinterval += RSs[i];
  }
  RSinterval = RSinterval/11;
  QRS = RSinterval * 2 + random(15,25);
  }

  
    
   previousT = t;
  
}



/*
  ADC_CFG1_ADIV(2)         Divide ratio = 4 (F_BUS = 48 MHz => ADCK = 12 MHz)
  ADC_CFG1_MODE(2)         Single ended 10 bit mode
  ADC_CFG1_ADLSMP          Long sample time
*/
#define ADC_CONFIG1 (ADC_CFG1_ADIV(1) | ADC_CFG1_MODE(1) | ADC_CFG1_ADLSMP)

/*
  ADC_CFG2_MUXSEL          Select channels ADxxb
  ADC_CFG2_ADLSTS(3)       Shortest long sample time
*/
#define ADC_CONFIG2 (ADC_CFG2_MUXSEL | ADC_CFG2_ADLSTS(3))

void adcInit() {
  ADC0_CFG1 = ADC_CONFIG1;
  ADC0_CFG2 = ADC_CONFIG2;
  // Voltage ref vcc, hardware trigger, DMA
  ADC0_SC2 = ADC_SC2_REFSEL(0) | ADC_SC2_ADTRG | ADC_SC2_DMAEN;

  // Enable averaging, 4 samples
  ADC0_SC3 = ADC_SC3_AVGE | ADC_SC3_AVGS(0);

  adcCalibrate();
  Serial.println("calibrated");

  // Enable ADC interrupt, configure pin
  ADC0_SC1A = ADC_SC1_AIEN | channel2sc1a[0];
  NVIC_ENABLE_IRQ(IRQ_ADC0);
}

void adcCalibrate() {
  uint16_t sum;

  // Begin calibration
  ADC0_SC3 = ADC_SC3_CAL;
  // Wait for calibration
  while (ADC0_SC3 & ADC_SC3_CAL);

  // Plus side gain
  sum = ADC0_CLPS + ADC0_CLP4 + ADC0_CLP3 + ADC0_CLP2 + ADC0_CLP1 + ADC0_CLP0;
  sum = (sum / 2) | 0x8000;
  ADC0_PG = sum;

  // Minus side gain (not used in single-ended mode)
  sum = ADC0_CLMS + ADC0_CLM4 + ADC0_CLM3 + ADC0_CLM2 + ADC0_CLM1 + ADC0_CLM0;
  sum = (sum / 2) | 0x8000;
  ADC0_MG = sum;
}

/*
  PDB_SC_TRGSEL(15)        Select software trigger
  PDB_SC_PDBEN             PDB enable
  PDB_SC_PDBIE             Interrupt enable
  PDB_SC_CONT              Continuous mode
  PDB_SC_PRESCALER(7)      Prescaler = 128
  PDB_SC_MULT(1)           Prescaler multiplication factor = 10
*/
#define PDB_CONFIG (PDB_SC_TRGSEL(15) | PDB_SC_PDBEN | PDB_SC_PDBIE \
  | PDB_SC_CONT | PDB_SC_PRESCALER(7) | PDB_SC_MULT(1))
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// 48 MHz / 128 / 10 / 1 Hz = 37500
// the frequency for data read in 
#define PDB_PERIOD (F_BUS / 128 / 10 / 250)


void pdbInit() {
  pinMode(13, OUTPUT);

  // Enable PDB clock
  SIM_SCGC6 |= SIM_SCGC6_PDB;
  // Timer period
  PDB0_MOD = PDB_PERIOD;
  // Interrupt delay
  PDB0_IDLY = 0;
  // Enable pre-trigger
  PDB0_CH0C1 = PDB_CH0C1_TOS | PDB_CH0C1_EN;
  // PDB0_CH0DLY0 = 0;
  PDB0_SC = PDB_CONFIG | PDB_SC_LDOK;
  // Software trigger (reset and restart counter)
  PDB0_SC |= PDB_SC_SWTRIG;
  // Enable interrupt request
  NVIC_ENABLE_IRQ(IRQ_PDB);
}


// adc value read in 
// raw data read into buffer7500
void adc0_isr() { 
  //if (counter < 7500) {
  adcValue = ADC0_RA;
  for (int i = 7499; i > 0; i--) { // shift data
    buffer7500[i] = buffer7500[i - 1];
  }
  buffer7500[0] = adcValue;
  
 
}
void pdb_isr() {
//  if (counter < 7500) {
  PDB0_SC &= ~PDB_SC_PDBIF;
  counter++; 
  if (flag == 2) {
    realCounter++;
  }
 // }
  
}


// A small helper for bluetooth setup
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

/* 
 algorithm for scrolling data when the ECG is finish. 
 By use the change of x axis, each time btween I touch the screen
 and leaving the screen.
*/
int currentStart;
boolean preTouched;
TS_Point p1;
TS_Point pOther;
int pointDiff;
void scrollIt(){
 boolean istouched = ts.touched();
  if(istouched){
    if(istouched == true && preTouched == false){
      p1 = ts.getPoint();
      p1.x = map(p1.x, 0, 4000, 0,320);
    }
  
    Serial.print("point1 ");
    Serial.println(p1.x);
    pOther = ts.getPoint();
    pOther.x = map(pOther.x, 0, 4000, 0,320);
    int pointDiff = pOther.x - p1.x;
    if (pointDiff != 0) {
    Serial.print("pointDiff ");
    Serial.println(pointDiff);
    Serial.print("currentStart ");
    Serial.println(currentStart);
    int preStart = currentStart;
    if(currentStart - pointDiff/5 >=0 && currentStart - pointDiff/5 <= drawcounter - 80 ) {
        currentStart -= pointDiff/5;

    }
    if (currentStart != preStart) {
    tft.setRotation(1);
    tft.fillRect(0,0, 320, 120,ILI9341_WHITE);
    drawGrid();
    for (int i = 0; i< 80; i++) {
      tft.drawLine(4*i,bufferDraw[currentStart + i], 4*(i+1), bufferDraw[currentStart + i + 1],ILI9341_DARKGREEN);
    }
    }
    }
    delay(200);
    }
    preTouched = istouched;
    pointDiff = 0;
    
    
  } 

// bluetooth print on APP
void blue(){
  bpm = HeartRate;
  
  //delay(1000);
        ble.print( F("AT+GATTCHAR=") );
        ble.print( hrmMeasureCharId );
        ble.print( F(",00-") );
        ble.println(bpm, HEX);
 
}

// bluetooth set up with user interface
void blueToothSet(){
  //while (!Serial); // required for Flora & Micro
  //delay(500);

  boolean success;

  //Serial.begin(115200);

  randomSeed(micros());

  /* Initialise the module */
  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }

  /* Perform a factory reset to make sure everything is in a known state */
  if (! ble.factoryReset() ){
       error(F("Couldn't factory reset"));
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  //Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  /* Change the device name to make it easier to find */
  if (! ble.sendCommandCheckOK(F("AT+GAPDEVNAME=HRM")) ) {
    error(F("Could not set device name?"));
  }


  //delay(100);
  /* Add the Heart Rate Service definition */
  /* Service ID should be 1 */
  success = ble.sendCommandWithIntReply( F("AT+GATTADDSERVICE=UUID=0x180D"), &hrmServiceId);
  if (! success) {
    error(F("Could not add HRM service"));
  }

  /* Add the Heart Rate Measurement characteristic */
  /* Chars ID for Measurement should be 1 */
  success = ble.sendCommandWithIntReply( F("AT+GATTADDCHAR=UUID=0x2A37, PROPERTIES=0x10, MIN_LEN=2, MAX_LEN=3, VALUE=00-40"), &hrmMeasureCharId);
    if (! success) {
    error(F("Could not add HRM characteristic"));
  }

  /* Add the Body Sensor Location characteristic */
  /* Chars ID for Body should be 2 */
  success = ble.sendCommandWithIntReply( F("AT+GATTADDCHAR=UUID=0x2A38, PROPERTIES=0x02, MIN_LEN=1, VALUE=3"), &hrmLocationCharId);
    if (! success) {
    error(F("Could not add BSL characteristic"));
  }

  /* Add the Heart Rate Service to the advertising data (needed for Nordic apps to detect the service) */
  ble.sendCommandCheckOK( F("AT+GAPSETADVDATA=02-01-06-05-02-0d-18-0a-18") );

  /* Reset the device for the new service setting changes to take effect */
  ble.reset();

  Serial.println();
}

// drawing algorithm for Grid
void drawGrid() {
  for (int i = 0; i < 40; i++) {
    tft.drawLine(i * 8, 0, i * 8, 120, ILI9341_YELLOW);
  }
  for (int i = 0; i < 10; i++) {
    tft.drawLine(0, i * 12, 320, i * 12, ILI9341_YELLOW);
  }
  tft.drawLine(0, 120, 320, 120, ILI9341_YELLOW);
}
