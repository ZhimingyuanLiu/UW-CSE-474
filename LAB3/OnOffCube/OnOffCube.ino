/*
  Draws a 3d rotating cube on the ILI9341 screen.
 */
#include <ILI9341_t3.h>
#include <font_Arial.h> // from ILI9341_t3
#include <XPT2046_Touchscreen.h>
#include <SPI.h>

#define CS_PIN  8
#define TFT_DC  9
#define TFT_CS 10
// MOSI=11, MISO=12, SCK=13
#include "SPI.h"
#include <Wire.h>
//#include <Adafruit_GFX.h>
#include "ILI9341_t3.h"

// For the Adafruit shield, these are the default.
#define TFT_DC 9
#define TFT_CS 10

// Use hardware SPI (on Uno, #13, #12, #11) and the above for CS/DC
ILI9341_t3 tft = ILI9341_t3(TFT_CS, TFT_DC);

// Define registers per MPU6050, Register Map and Descriptions, Rev 4.2, 08/19/2013 6 DOF Motion sensor fusion device
// Invensense Inc., www.invensense.com
// See also MPU-6050 Register Map and Descriptions, Revision 4.0, RM-MPU-6050A-00, 9/12/2012 for registers not listed in 
// above document; the MPU6050 and MPU-9150 are virtually identical but the latter has an on-board magnetic sensor
//
#define XGOFFS_TC        0x00 // Bit 7 PWR_MODE, bits 6:1 XG_OFFS_TC, bit 0 OTP_BNK_VLD                 
#define YGOFFS_TC        0x01                                                                          
#define ZGOFFS_TC        0x02
#define X_FINE_GAIN      0x03 // [7:0] fine gain
#define Y_FINE_GAIN      0x04
#define Z_FINE_GAIN      0x05
#define XA_OFFSET_H      0x06 // User-defined trim values for accelerometer
#define XA_OFFSET_L_TC   0x07
#define YA_OFFSET_H      0x08
#define YA_OFFSET_L_TC   0x09
#define ZA_OFFSET_H      0x0A
#define ZA_OFFSET_L_TC   0x0B
#define SELF_TEST_X      0x0D
#define SELF_TEST_Y      0x0E    
#define SELF_TEST_Z      0x0F
#define SELF_TEST_A      0x10
#define XG_OFFS_USRH     0x13  // User-defined trim values for gyroscope; supported in MPU-6050?
#define XG_OFFS_USRL     0x14
#define YG_OFFS_USRH     0x15
#define YG_OFFS_USRL     0x16
#define ZG_OFFS_USRH     0x17
#define ZG_OFFS_USRL     0x18
#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define FF_THR           0x1D  // Free-fall
#define FF_DUR           0x1E  // Free-fall
#define MOT_THR          0x1F  // Motion detection threshold bits [7:0]
#define MOT_DUR          0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define ZMOT_THR         0x21  // Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR        0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms
#define FIFO_EN          0x23
#define I2C_MST_CTRL     0x24   
#define I2C_SLV0_ADDR    0x25
#define I2C_SLV0_REG     0x26
#define I2C_SLV0_CTRL    0x27
#define I2C_SLV1_ADDR    0x28
#define I2C_SLV1_REG     0x29
#define I2C_SLV1_CTRL    0x2A
#define I2C_SLV2_ADDR    0x2B
#define I2C_SLV2_REG     0x2C
#define I2C_SLV2_CTRL    0x2D
#define I2C_SLV3_ADDR    0x2E
#define I2C_SLV3_REG     0x2F
#define I2C_SLV3_CTRL    0x30
#define I2C_SLV4_ADDR    0x31
#define I2C_SLV4_REG     0x32
#define I2C_SLV4_DO      0x33
#define I2C_SLV4_CTRL    0x34
#define I2C_SLV4_DI      0x35
#define I2C_MST_STATUS   0x36
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define DMP_INT_STATUS   0x39  // Check DMP interrupt
#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40
#define TEMP_OUT_H       0x41
#define TEMP_OUT_L       0x42
#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60
#define MOT_DETECT_STATUS 0x61
#define I2C_SLV0_DO      0x63
#define I2C_SLV1_DO      0x64
#define I2C_SLV2_DO      0x65
#define I2C_SLV3_DO      0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL   0x69
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C
#define DMP_BANK         0x6D  // Activates a specific bank in the DMP
#define DMP_RW_PNT       0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG          0x6F  // Register in DMP from which to read or to which to write
#define DMP_REG_1        0x70
#define DMP_REG_2        0x71
#define FIFO_COUNTH      0x72
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74
#define WHO_AM_I_MPU6050 0x75 // Should return 0x68

// Using the GY-521 breakout board, I set ADO to 0 by grounding through a 4k7 resistor
// Seven-bit device address is 110100 for ADO = 0 and 110101 for ADO = 1
#define ADO 0
#if ADO
#define MPU6050_ADDRESS 0x69  // Device address when ADO = 1
#else
#define MPU6050_ADDRESS 0x68  // Device address when ADO = 0
#endif

// Set initial input parameters
enum Ascale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

enum Gscale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

// Specify sensor full scale
int Gscale = GFS_250DPS;
int Ascale = AFS_2G;
float aRes, gRes; // scale resolutions per LSB for the sensors
  
// Pin definitions
int intPin = 12;  // This can be changed, 2 and 3 are the Arduinos ext int pins

int16_t accelCount[3];           // Stores the 16-bit signed accelerometer sensor output
float ax, ay, az;                // Stores the real accel value in g's
int16_t gyroCount[3];            // Stores the 16-bit signed gyro sensor output
float gx, gy, gz;                // Stores the real gyro value in degrees per seconds
float gyroBias[3], accelBias[3]; // Bias corrections for gyro and accelerometer
int16_t tempCount;               // Stores the internal chip temperature sensor output 
float temperature;               // Scaled temperature in degrees Celsius
float SelfTest[6];               // Gyro and accelerometer self-test sensor output
uint32_t count = 0;


const int numReadings = 5;
int readIndexAx = 0;              // the index of the current reading
int readIndexAy = 0;
int readIndexAz = 0;
int readIndexGx = 0;              // the index of the current reading
int readIndexGy = 0;
int readIndexGz = 0;

int readingsAx[numReadings];      // the readings from the analog input
int totalAx = 0;                  // the running total
int averageAx = 0;                // the average

int readingsAy[numReadings];      // the readings from the analog input
int totalAy = 0;                  // the running total
int averageAy = 0;                // the average

int readingsAz[numReadings];      // the readings from the analog input
int totalAz = 0;                  // the running total
int averageAz = 0;                // the average



int readingsGx[numReadings];      // the readings from the analog input
int totalGx = 0;                  // the running total
int averageGx = 0;                // the average

int readingsGy[numReadings];      // the readings from the analog input
int totalGy = 0;                  // the running total
int averageGy = 0;                // the average

int readingsGz[numReadings];      // the readings from the analog input
int totalGz = 0;                  // the running total
int averageGz = 0;                // the average



XPT2046_Touchscreen ts(CS_PIN);
#define TIRQ_PIN  2
//XPT2046_Touchscreen ts(CS_PIN);  // Param 2 - NULL - No interrupts
//XPT2046_Touchscreen ts(CS_PIN, 255);  // Param 2 - 255 - No interrupts
//XPT2046_Touchscreen ts(CS_PIN, TIRQ_PIN);  // Param 2 - Touch IRQ Pin - interrupt enabled polling


typedef struct
{
  uint16_t red   : 5;
  uint16_t green : 6;
  uint16_t blue  : 5;
} Color;


uint16_t Col;
Color PIX;


#define ScreenColor ILI9341_YELLOW
#define halfline 40

float angle;
float xx,xy,xz;
float yx,yy,yz;
float zx,zy,zz;

float fact;

int Xan,Yan;

int Xoff;
int Yoff;
int Zoff;

struct Point3d
{
  int x;
  int y;
  int z;
};

struct Point2d
{
  int x;
  int y;
};

int LinestoRender; // lines to render.
int OldLinestoRender; // lines to render just in case it changes. this makes sure the old lines all get erased.
int readIndexAx = 0;              // the index of the current reading
int readIndexAy = 0;
int readIndexAz = 0;
int readIndexGx = 0;              // the index of the current reading
int readIndexGy = 0;
int readIndexGz = 0;

int readingsAx[numReadings];      // the readings from the analog input
int totalAx = 0;                  // the running total
int averageAx = 0;                // the average
long MappedAx;

int readingsAy[numReadings];      // the readings from the analog input
int totalAy = 0;                  // the running total
int averageAy = 0;                // the average
long MappedAy;

int readingsAz[numReadings];      // the readings from the analog input
int totalAz = 0;                  // the running total
int averageAz = 0;                // the average
long MappedAz;


int readingsGx[numReadings];      // the readings from the analog input
int totalGx = 0;                  // the running total
int averageGx = 0;                // the average
long MappedGx;

int readingsGy[numReadings];      // the readings from the analog input
int totalGy = 0;                  // the running total
int averageGy = 0;                // the average
long MappedGy;

int readingsGz[numReadings];      // the readings from the analog input
int totalGz = 0;                  // the running total
int averageGz = 0;                // the average
long MappedGz;

struct Line3d
{
  Point3d p0;
  Point3d p1;
};

struct Line2d
{
  Point2d p0;
  Point2d p1;
};

Line3d Lines[20];
Line2d Render[20];
Line2d ORender[20];

/***********************************************************************************************************************************/
// Sets the global vars for the 3d transform. Any points sent through "process" will be transformed using these figures.
// only needs to be called if Xan or Yan are changed.
void SetVars(void)
{
  
  float Xan2,Yan2,Zan2;
  float s1,s2,s3,c1,c2,c3;
  //int val = analogRead(2);
  Xan2 = Xan / fact; // convert degrees to radians.
  Yan2 = Yan / fact;

  // Zan is assumed to be zero

  s1 = sin(Yan2);
  s2 = sin(Xan2);

  c1 = cos(Yan2);
  c2 = cos(Xan2);

  xx = c1*MappedGz;
  xy = 0*MappedGz;
  xz = -s1*MappedGz;

  yx = (s1 * s2)*;
  yy = c2*MappedGz;
  yz = (c1 * s2)*MappedGz);

  zx = (s1 * c2)*MappedGz;
  zy = -s2*MappedGz;
  zz = (c1 * c2)*MappedGz;
}
/***********************************************************************************************************************************/
// processes x1,y1,z1 and returns rx1,ry1 transformed by the variables set in SetVars()
// fairly heavy on floating point here.
// uses a bunch of global vars. Could be rewritten with a struct but possibly not worth the effort.
void ProcessLine(struct Line2d *ret,struct Line3d vec)
{
  float zvt1;
  int xv1,yv1,zv1;

  float zvt2;
  int xv2,yv2,zv2;

  int rx1,ry1;
  int rx2,ry2;

  int x1;
  int y1;
  int z1;

  int x2;
  int y2;
  int z2;

  int Ok;

  x1=vec.p0.x;
  y1=vec.p0.y;
  z1=vec.p0.z;

  x2=vec.p1.x;
  y2=vec.p1.y;
  z2=vec.p1.z;

  Ok=0; // defaults to not OK

  xv1 = (x1 * xx) + (y1 * xy) + (z1 * xz);
  yv1 = (x1 * yx) + (y1 * yy) + (z1 * yz);
  zv1 = (x1 * zx) + (y1 * zy) + (z1 * zz);

  zvt1 = zv1 - Zoff;


  if( zvt1 < -5){
    rx1 = 256 * (xv1 / zvt1) + Xoff;
    ry1 = 256 * (yv1 / zvt1) + Yoff;
    Ok=1; // ok we are alright for point 1.
  }

  xv2 = (x2 * xx) + (y2 * xy) + (z2 * xz);
  yv2 = (x2 * yx) + (y2 * yy) + (z2 * yz);
  zv2 = (x2 * zx) + (y2 * zy) + (z2 * zz);

  zvt2 = zv2 - Zoff;

  if( zvt2 < -5){
    rx2 = 256 * (xv2 / zvt2) + Xoff;
    ry2 = 256 * (yv2 / zvt2) + Yoff;
  } 
  else
  {
    Ok=0;
  }

  if(Ok==1){

    ret->p0.x=rx1;
    ret->p0.y=ry1;

    ret->p1.x=rx2;
    ret->p1.y=ry2;
  }
  // The ifs here are checks for out of bounds. needs a bit more code here to "safe" lines 
  //that will be way out of whack, so they dont get drawn and cause screen garbage.
}

/***********************************************************************************************************************************/
void setup() {
  Wire.begin();
  tft.begin();
  Serial.begin(9600);
  
// in here is specific code for the display. change if you are using a different screen.
  tft.fillScreen(ScreenColor);

  fact = 180 / 3.14159259; // conversion from degrees to radians.
  //int val = analogRead(2);
  
  Xoff = 120; // positions the center of the 3d conversion space into the center of the screen. This is usally screen_x_size / 2.
  Yoff = 160; // screen_y_size /2
  Zoff = 500;
  // line segments to draw a cube. basically p0 to p1. p1 to p2. p2 to p3 so on.
  // Front Face.

  Lines[0].p0.x=-halfline;
  Lines[0].p0.y=-halfline;
  Lines[0].p0.z=halfline;
  Lines[0].p1.x=halfline;
  Lines[0].p1.y=-halfline;
  Lines[0].p1.z=halfline;

  Lines[1].p0.x=halfline;
  Lines[1].p0.y=-halfline;
  Lines[1].p0.z=halfline;
  Lines[1].p1.x=halfline;
  Lines[1].p1.y=halfline;
  Lines[1].p1.z=halfline;

  Lines[2].p0.x=halfline;
  Lines[2].p0.y=halfline;
  Lines[2].p0.z=halfline;
  Lines[2].p1.x=-halfline;
  Lines[2].p1.y=halfline;
  Lines[2].p1.z=halfline;

  Lines[3].p0.x=-halfline;
  Lines[3].p0.y=halfline;
  Lines[3].p0.z=halfline;
  Lines[3].p1.x=-halfline;
  Lines[3].p1.y=-halfline;
  Lines[3].p1.z=halfline;

  //back face.

  Lines[4].p0.x=-halfline;
  Lines[4].p0.y=-halfline;
  Lines[4].p0.z=-halfline;
  Lines[4].p1.x=halfline;
  Lines[4].p1.y=-halfline;
  Lines[4].p1.z=-halfline;

  Lines[5].p0.x=halfline;
  Lines[5].p0.y=-halfline;
  Lines[5].p0.z=-halfline;
  Lines[5].p1.x=halfline;
  Lines[5].p1.y=halfline;
  Lines[5].p1.z=-halfline;

  Lines[6].p0.x=halfline;
  Lines[6].p0.y=halfline;
  Lines[6].p0.z=-halfline;
  Lines[6].p1.x=-halfline;
  Lines[6].p1.y=halfline;
  Lines[6].p1.z=-halfline;

  Lines[7].p0.x=-halfline;
  Lines[7].p0.y=halfline;
  Lines[7].p0.z=-halfline;
  Lines[7].p1.x=-halfline;
  Lines[7].p1.y=-halfline;
  Lines[7].p1.z=-halfline;

  // now the 4 edge lines.

  Lines[8].p0.x=-halfline;
  Lines[8].p0.y=-halfline;
  Lines[8].p0.z=halfline;
  Lines[8].p1.x=-halfline;
  Lines[8].p1.y=-halfline;
  Lines[8].p1.z=-halfline;

  Lines[9].p0.x=halfline;
  Lines[9].p0.y=-halfline;
  Lines[9].p0.z=halfline;
  Lines[9].p1.x=halfline;
  Lines[9].p1.y=-halfline;
  Lines[9].p1.z=-halfline;

  Lines[10].p0.x=-halfline;
  Lines[10].p0.y=halfline;
  Lines[10].p0.z=halfline;
  Lines[10].p1.x=-halfline;
  Lines[10].p1.y=halfline;
  Lines[10].p1.z=-halfline;

  Lines[11].p0.x=halfline;
  Lines[11].p0.y=halfline;
  Lines[11].p0.z=halfline;
  Lines[11].p1.x=halfline;
  Lines[11].p1.y=halfline;
  Lines[11].p1.z=-halfline;

  LinestoRender=12;
  OldLinestoRender=LinestoRender;
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readingsAx[thisReading] = 0;
  }

  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readingsAy[thisReading] = 0;
  }

  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readingsAz[thisReading] = 0;
  }
  
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readingsGx[thisReading] = 0;
  }
  
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readingsGy[thisReading] = 0;
  }
 
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readingsGz[thisReading] = 0;
  }
  
  
  
  
  
  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(intPin, INPUT);
  digitalWrite(intPin, LOW);
  
  
  
  tft.begin();         // Initialize the display
 // tft.setContrast(58); // Set the contrast
  tft.setRotation(2);  //  0 or 2) width = width, 1 or 3) width = height, swapped etc.

  
// Start device display with ID of sensor
  tft.fillScreen(ILI9341_WHITE);
  tft.setTextColor(ILI9341_BLACK); // Set pixel color; 1 on the monochrome screen
  tft.setTextSize(2);
  tft.setCursor(40,0); tft.print("MPU6050");
  //tft.setTextSize(1);
  tft.setCursor(0, 40); tft.print("6-DOF 16-bit");
  tft.setCursor(0, 60); tft.print("motion sensor");
  tft.setCursor(40,80); tft.print("60 ug LSB");
//   
  delay(1000);

// Set up for data display

  tft.fillScreen(ILI9341_WHITE);      // clears the screen and buffer
  tft.setCursor(20,0); tft.print("   6050");

  // Read the WHO_AM_I register, this is a good test of communication
  uint8_t c = readByte(MPU6050_ADDRESS, WHO_AM_I_MPU6050);  // Read WHO_AM_I register for MPU-6050
  tft.setCursor(20,0); tft.print("MPU6050");
  tft.setCursor(0,20); tft.print("I AM");
  tft.setCursor(0,40); tft.print(c, HEX);  
  tft.setCursor(0,60); tft.print("I Should Be");
  tft.setCursor(0,80); tft.print(0x68, HEX); 
//   
  delay(1000); 

  if (c == 0x68) // WHO_AM_I should always be 0x68
  {  
    Serial.println("MPU6050 is online...");
    
    MPU6050SelfTest(SelfTest); // Start by performing self test and reporting values
    Serial.print("x-axis self test: acceleration trim within : "); Serial.print(SelfTest[0],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: acceleration trim within : "); Serial.print(SelfTest[1],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: acceleration trim within : "); Serial.print(SelfTest[2],1); Serial.println("% of factory value");
    Serial.print("x-axis self test: gyration trim within : "); Serial.print(SelfTest[3],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: gyration trim within : "); Serial.print(SelfTest[4],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: gyration trim within : "); Serial.print(SelfTest[5],1); Serial.println("% of factory value");

    if(SelfTest[0] < 1.0f && SelfTest[1] < 1.0f && SelfTest[2] < 1.0f && SelfTest[3] < 1.0f && SelfTest[4] < 1.0f && SelfTest[5] < 1.0f) {
//    tft.clearDisplay();
    tft.setCursor(0, 120); tft.print("Pass Selftest!");  
//     
    delay(1000);
  
    calibrateMPU6050(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers  
    initMPU6050(); Serial.println("MPU6050 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
//  tft.fillScreen(ILI9341_WHITE);
    tft.setTextSize(2);
   }
   else
   {
    Serial.print("Could not connect to MPU6050: 0x");
    Serial.println(c, HEX);
    while(1) ; // Loop forever if communication doesn't happen
   }

  }
}

/***********************************************************************************************************************************/
void RenderImage( void)
{
  // renders all the lines after erasing the old ones.
  // in here is the only code actually interfacing with the screen. so if you use a different lib, this is where to change it.

  for (int i=0; i<OldLinestoRender; i++ )
  {
    tft.drawLine(ORender[i].p0.x,ORender[i].p0.y,ORender[i].p1.x,ORender[i].p1.y, ScreenColor); // erase the old lines.
  }

  for (int i=0; i<LinestoRender; i++ )
  {
    tft.drawLine(Render[i].p0.x,Render[i].p0.y,Render[i].p1.x,Render[i].p1.y, *(uint16_t*)&PIX);
  }
  OldLinestoRender=LinestoRender;
}

// Translate a hue "angle" -120 to 120 degrees (ie -2PI/3 to 2PI/3) to
// a 6-bit R channel value
//
inline int angle_to_channel(float a) {
  if(a < -PI)
    a += 2*PI;
  if(a < -2*PI/3  || a > 2*PI/3)
    return 0;
  float f_channel = cos(a*3/4); // remap 120-degree 0-1.0 to 90 ??
  return ceil(f_channel * 63);
}

/***********************************************************************************************************************************/
int flag = 0;
void loop() {
 
  if(readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01) {  // check if data ready interrupt

    readAccelData(accelCount);  // Read the x/y/z adc values
    getAres();
    
    // Now we'll calculate the accleration value into actual g's
    ax = (float)accelCount[0]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
    ay = (float)accelCount[1]*aRes - accelBias[1];   
    az = (float)accelCount[2]*aRes - accelBias[2];  
   
    readGyroData(gyroCount);  // Read the x/y/z adc values
    getGres();
 
    // Calculate the gyro value into actual degrees per second
    gx = (float)gyroCount[0]*gRes - gyroBias[0];  // get actual gyro value, this depends on scale being set
    gy = (float)gyroCount[1]*gRes - gyroBias[1];  
    gz = (float)gyroCount[2]*gRes - gyroBias[2];   

    tempCount = readTempData();  // Read the x/y/z adc values
    temperature = ((float) tempCount) / 340. + 36.53; // Temperature in degrees Centigrade
   }  
  

  totalAx = totalAx - readingsAx[readIndexAx];
  // read from the sensor:
  readingsAx[readIndexAx] = ax*1000;
  // add the reading to the total:
  totalAx = totalAx + readingsAx[readIndexAx];
  // advance to the next position in the array:
  readIndexAx = readIndexAx + 1;
  // if we're at the end of the array...
  if (readIndexAx >= numReadings) {
    // ...wrap around to the beginning:
    readIndexAx = 0;
  }
  // calculate the average:
  averageAx = totalAx / numReadings;
  
  MappedAx = map(averageAx, -2000, 2000, -80, 80);
  
  totalAy = totalAy - readingsAy[readIndexAy];
  readingsAy[readIndexAy] = ay*1000;
  // add the reading to the total:
  totalAy = totalAy + readingsAy[readIndexAy];
  // advance to the next position in the array:
  readIndexAy = readIndexAy + 1;
  // if we're at the end of the array...
  if (readIndexAy >= numReadings) {
    // ...wrap around to the beginning:
    readIndexAy = 0;
  }
  // calculate the average:
  averageAy = totalAy / numReadings;
  MappedAy = map(averageAy, -2000, 2000, -80, 80);

  totalAz = totalAz - readingsAz[readIndexAz];
  readingsAz[readIndexAz] = az*1000;
  // add the reading to the total:
  totalAz = totalAz + readingsAz[readIndexAz];
  // advance to the next position in the array:
  readIndexAz = readIndexAz + 1;
  // if we're at the end of the array...
  if (readIndexAz >= numReadings) {
    // ...wrap around to the beginning:
    readIndexAz = 0;
  }
  // calculate the average:
  averageAz = totalAz / numReadings;
  MappedAz = map(averageAz, -2000, 2000, -80, 80);



    totalGx = totalGx - readingsGx[readIndexGx];
  // read from the sensor:
  readingsGx[readIndexGx] = gx;
  // add the reading to the total:
  totalGx = totalGx + readingsGx[readIndexGx];
  // advance to the next position in the array:
  readIndexGx = readIndexGx + 1;
  // if we're at the end of the array...
  if (readIndexGx >= numReadings) {
    // ...wrap around to the beginning:
    readIndexGx = 0;
  }
  // calculate the average:
  averageGx = totalGx / numReadings;
  MappedGx = map(averageGx, -150, 150, -80, 80);
  
  
  totalGy = totalGy - readingsGy[readIndexGy];
  readingsGy[readIndexGy] = gy;
  // add the reading to the total:
  totalGy = totalGy + readingsGy[readIndexGy];
  // advance to the next position in the array:
  readIndexGy = readIndexGy + 1;
  // if we're at the end of the array...
  if (readIndexGy >= numReadings) {
    // ...wrap around to the beginning:
    readIndexGy = 0;
  }
  // calculate the average:
  averageGy = totalGy / numReadings;
  MappedGy = map(averageGy, -150, 150, -80, 80);


  totalGz = totalGz - readingsGz[readIndexGz];
  readingsGz[readIndexGz] = gz;
  // add the reading to the total:
  totalGz = totalGz + readingsGz[readIndexGz];
  // advance to the next position in the array:
  readIndexGz = readIndexGz + 1;
  // if we're at the end of the array...
  if (readIndexGz >= numReadings) {
    // ...wrap around to the beginning:
    readIndexGz = 0;
  }
  // calculate the average:
  averageGz = totalGz / numReadings;
  MappedGz = map(averageGz, -150, 150, -80, 80);
  
  PIX.red = angle_to_channel(angle)>>1;
  PIX.green = angle_to_channel(angle-2*PI/3);
  PIX.blue = angle_to_channel(angle-4*PI/3)>>1;
  
  angle += 0.02;
  if(angle > PI)
    angle -= 2*PI;
    
  Xan++;
  Yan++;

  Yan=Yan % 360;
  Xan=Xan % 360; // prevents overflow.

  SetVars(); //sets up the global vars to do the conversion.

  for(int i=0; i<LinestoRender ; i++)
  {
    ORender[i]=Render[i]; // stores the old line segment so we can delete it later.
    ProcessLine(&Render[i],Lines[i]); // converts the 3d line segments to 2d.
  } 

  RenderImage(); // go draw it!
  Serial.println(*(uint16_t*)&PIX,HEX);
  delay(40);
}
  

  
 

  SetVars(); //sets up the global vars to do the conversion.
 //}
  for(int i=0; i<LinestoRender ; i++)
  {
    ORender[i]=Render[i]; // stores the old line segment so we can delete it later.
    ProcessLine(&Render[i],Lines[i]); // converts the 3d line segments to 2d.
  } 

  RenderImage(); // go draw it!
  Serial.println(*(uint16_t*)&PIX,HEX);
  delay(40);


}
/***********************************************************************************************************************************/

