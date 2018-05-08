// FFT Test
//
// Compute a 1024 point Fast Fourier Transform (spectrum analysis)
// on audio connected to the Left Line-In pin.  By changing code,
// a synthetic sine wave can be input instead.
//
// The first 40 (of 512) frequency analysis bins are printed to
// the Arduino Serial Monitor.  Viewing the raw data can help you
// understand how the FFT works and what results to expect when
// using the data to control LEDs, motors, or other fun things!
//
// This example code is in the public domain.

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
//#include <SerialFlash.h>

const int myInput = AUDIO_INPUT_LINEIN;
int freq;
//const int myInput = AUDIO_INPUT_MIC;

// Create the Audio components.  These should be created in the
// order data flows, inputs/sources -> processing -> outputs

// GUItool: begin automatically generated code
AudioSynthWaveformSine   sine1;          //xy=249,132
AudioInputAnalog         adc1;           //xy=251,179
AudioOutputAnalog        dac1;           //xy=551,183
AudioAnalyzeFFT1024      fft1024_1;      //xy=560,110
// Connect either the live input or synthesized sine wave
//AudioConnection          patchCord1(adc1, fft1024_1);
//AudioConnection          patchCord2(adc1, dac1);
AudioConnection          patchCord1(sine1, fft1024_1);
AudioConnection          patchCord2(sine1, dac1);
// GUItool: end automatically generated code



//AudioControlSGTL5000 audioShield;

void setup() {
  // Audio connections require memory to work.  For more
  // detailed information, see the MemoryAndCpuUsage example
  AudioMemory(12);

  // Enable the audio shield and set the output volume.
//  audioShield.enable();
//  audioShield.inputSelect(myInput);
//  audioShield.volume(0.5);

  // Configure the window algorithm to use
  fft1024_1.windowFunction(AudioWindowHanning1024);
  //fft1024_1.windowFunction(NULL);

  // Create a synthetic sine wave, for testing
  // To use this, edit the connections above
  sine1.amplitude(0.8);
  sine1.frequency(freq);
}

void loop() {
 
  float n;
  int i;
  
  if (fft1024_1.available()) {
    // each time new FFT data is available
    // print it all to the Arduino Serial Monitor
    Serial.print("FFT: ");
    for (i=0; i<16; i++) {
       n = fft1024_1.read(i*32, (i*32)+31);
      if (n >= 0.01) {
        Serial.print(n);
        Serial.print(" ");
      } else {
        Serial.print("  -  "); // don't print "0.00"
      }
    }
    Serial.println();
  }
   if(freq < 20000) {
    freq += 20;
  } else {
    freq = 20;
  }
  sine1.frequency(freq);
}

