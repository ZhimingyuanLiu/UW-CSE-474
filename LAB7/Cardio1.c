/* ADCpdbDMA
PDB triggers the ADC which requests the DMA to move the data to a buffer
*/
#define PDB_CH0C1_TOS 0x0100
#define PDB_CH0C1_EN 0x01
#define TFT_DC  9
#define TFT_CS 10

#include <SD.h>
#include <SPI.h>
#include "SPI.h"
#include "ILI9341_t3.h"
#include <kinetis.h>
ILI9341_t3 tft = ILI9341_t3(TFT_CS, TFT_DC);

uint8_t samplesMap;
uint16_t samples;
uint16_t buffer[20];
//uint16_t buffer2[5]
int pix = 0;
int count = 0;
int dataRead;
float prev = 0;
float prevAvg = 0;
int ps = 1;
int ns = 1;
int screenOn;
bool move;


void ECG(){
  SamplesMap = map(dataRead, 0, 4086, 0, 240);
  buffer[0] = SamplesMap;
  tft.setRotation(2);
  tft.drawLine(pre, pix-1, avg(), pix, ILI9341_BLACK);
  pix++;
  prev = prevAvg;

  if (pix >= 319) { // wrap around to left side of screen
    createGrid();
    pix = 0;
  }
  for (int i = 1; i < 20; i++) { // shift data
    buffer[i] = buffer[i - 1];
  }
}


void createGrid() { // clear the screen and draw the grid
  tft.setRotation(1);
  tft.fillScreen(ILI9341_WHITE);
  for (int i = 0; i < 16; i++) {
    tft.drawLine(i*20, 0, i*20, 240, ILI9341_RED); // vertical lines
    if (i % 5 == 0) {
      tft.drawLine(i*20+1, 0, i*20+1, 240, ILI9341_RED);
    }
    if (i < 12) {
      tft.drawLine(0, i*20, 320, i*20, ILI9341_RED); // horizontal lines
      if (i % 5 == 0) {
        tft.drawLine(0, i*20 + 1, 320, i*20 + 1, ILI9341_RED);
      }
    }
  }
}


float avg(){ // take average of last five points
    float currentAvg = 0;
    for (int i = 0; i < 20; i++) {
      currentAvg += buffer[i]; 
    }
    prevAvg = currentAvg / 20;
    return currentAvg / 20;
}



void setup() {
  Serial.begin(9600);
  //while (!Serial); // wait until the serial debug window is opened
  pdbInit();
  tft.begin();
  tft.fillScreen(ILI9341_WHITE);
  tft.setRotation(1);
  tft.setTextSize(4);
  tft.setTextColor(ILI9341_BLACK);
  tft.setCursor(50,50);
  createGrid();
  adcInit();
  
  dmaInit();
}

void loop() {
}

static const uint8_t channel2sc1a[] = {
  5, 14, 8, 9, 13, 12, 6, 7, 15, 4,
  0, 19, 3, 21, 26, 22
};

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

// 48 MHz / 128 / 10 / 1 Hz = 37500
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


void adc0_isr() {
  //int sampleLast = samplesMap[63];
  Serial.print("adc isr: ");
  Serial.println(millis());
  dataRead = samples;
  
}



void pdb_isr() {
  //Serial.print("pdb isr: ");
  //Serial.println(millis());
  int sensor = digitalread(2);
  count++;
  if(count <= 7500 && move){
    switch(ps){
      case(1):
        if(sensor == 1){
          screenOn = 1;
          ECG();
          ns = 1;
          move = true;
        }else if (sensor == 0 && screenOn == 1){
          ns = 2;
          move = true;
        }else if (sensor == 0 && screenOn == 0){
          move = false;
        }
      
      case(2):
        if(sensor == 1){
          ns = 2; 
          ScreenOn = 0; 
          move = false;
        }else if (sensor == 0 && ScreenOn == 0){
          ns = 1;
          move = false;
        }else if (sensor == 0 && ScreenOn == 1){
          ECG();
          move = true;
        }
    }

    ps = ns;
    delay(10);
  }
}



void dma_ch1_isr() {
  //Serial.print("dma isr: ");
  //Serial.println(millis());
  // Clear interrupt request for channel 1
  DMA_CINT = 1;
}



void dmaInit() {
  // Enable DMA, DMAMUX clocks
  SIM_SCGC7 |= SIM_SCGC7_DMA;
  SIM_SCGC6 |= SIM_SCGC6_DMAMUX;

  // Use default configuration
  DMA_CR = 0;

  // Source address
  DMA_TCD1_SADDR = &ADC0_RA;
  // Don't change source address
  DMA_TCD1_SOFF = 0;
  DMA_TCD1_SLAST = 0;
  // Destination address
  DMA_TCD1_DADDR = samples;
  // Destination offset (2 byte)
  DMA_TCD1_DOFF = 2;
  // Restore destination address after major loop
  DMA_TCD1_DLASTSGA = -sizeof(samples);
  // Source and destination size 16 bit
  DMA_TCD1_ATTR = DMA_TCD_ATTR_SSIZE(1) | DMA_TCD_ATTR_DSIZE(1);
  // Number of bytes to transfer (in each service request)
  DMA_TCD1_NBYTES_MLNO = 2;
  // Set loop counts
  DMA_TCD1_CITER_ELINKNO = sizeof(samples) / 2;
  DMA_TCD1_BITER_ELINKNO = sizeof(samples) / 2;
  // Enable interrupt (end-of-major loop)
  DMA_TCD1_CSR = DMA_TCD_CSR_INTMAJOR;

  // Set ADC as source (CH 1), enable DMA MUX
  DMAMUX0_CHCFG1 = DMAMUX_DISABLE;
  DMAMUX0_CHCFG1 = DMAMUX_SOURCE_ADC0 | DMAMUX_ENABLE;

  // Enable request input signal for channel 1
  DMA_SERQ = 1;

  // Enable interrupt request
  NVIC_ENABLE_IRQ(IRQ_DMA_CH1);
}


