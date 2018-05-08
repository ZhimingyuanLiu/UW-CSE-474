float phase = 0.0;
float twopi = 3.14159 * 2;
elapsedMicros usec = 0;

void setup() {
  analogWriteResolution(12);
  analogReadResolution(12);
}

void loop() {
 // float val = sin(phase) * 2000.0 + 2050.0;
  analogWrite(A14, analogRead(A0));
  zdelayMicroseconds(50);

  //phase = phase + 0.02;
  //if (phase >= twopi) phase = 0;
  //while (usec < 500) ; // wait
  //usec = usec - 500;
}
