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
#include <SerialFlash.h>
#include <Math.h>

const int myInput = AUDIO_INPUT_LINEIN;
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
  sine1.frequency(100);
}

float stage[16];

void loop() {
  //float n;
  //int i;

 int i;

  if (fft1024_1.available()) {
    // each time new FFT data is available
    // print it all to the Arduino Serial Monitor    
    stage[0] =  fft1024_1.read(0);
    stage[1] =  fft1024_1.read(1);
    stage[2] =  fft1024_1.read(2, 4);
    stage[3] =  fft1024_1.read(5, 7);
    stage[4] =  fft1024_1.read(8, 11);
    stage[5] =  fft1024_1.read(12, 16);
    stage[6] =  fft1024_1.read(17, 23);
    stage[7] =  fft1024_1.read(24, 33);
    stage[8] =  fft1024_1.read(34, 47);
    stage[9] =  fft1024_1.read(48, 67);
    stage[10] = fft1024_1.read(68, 94);
    stage[11] = fft1024_1.read(95, 132);
    stage[12] = fft1024_1.read(133, 185);
    stage[13] = fft1024_1.read(186, 258);
    stage[14] = fft1024_1.read(259, 360);
    stage[15] = fft1024_1.read(361, 511);
    // each time new FFT data is available
    // print it all to the Arduino Serial Monitor
    Serial.print("FFT: ");
    for (i=0; i<16; i++) {
     // n = fft1024_1.read(i);
      if (stage[i] >= 0.01) {
        Serial.print(stage[i]);
        Serial.print(" ");
      } else {
        Serial.print("  -  "); // don't print "0.00"
      }
    }
    Serial.println();
  }
}
