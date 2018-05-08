// Create an IntervalTimer object 
IntervalTimer myTimer;

const int PinRed = 20;  // RED
const int PinGreen = 21;  // GREEN
const int PinBlue = 22;  // BLUE
void setup(void) {
  pinMode(PinRed, OUTPUT);
  pinMode(PinGreen, OUTPUT);
  pinMode(PinBlue, OUTPUT);
  Serial.begin(9600);
  myTimer.begin(blinkLED, 1000000);  // blinkLED to run every 1 seconds
}

// The interrupt will blink the LED, and keep
// track of how many times it has blinked.

int redState = HIGH;
int blueState = HIGH;
int greenState = HIGH;

volatile unsigned long blinkCount = 1; // use volatile for shared variables

// functions called by IntervalTimer should be short, run as quickly as
// possible, and should avoid calling other functions if possible.
void blinkLED(void) {
  if (redState == HIGH && greenState == HIGH && blueState == HIGH) {
    redState = LOW;
    blinkCount = 0;  // increase when LED turns on
  } else if(redState == LOW && greenState == HIGH && blueState == HIGH){
    redState = HIGH;
    greenState = LOW;
    blinkCount = 1;  // increase when LED turns on
  } else if(redState == HIGH && greenState == LOW && blueState == HIGH){
    greenState = HIGH;
    blueState = LOW;
    blinkCount = 2;  // increase when LED turns on
  } else if(redState == HIGH && greenState == HIGH && blueState == LOW){
    blueState = HIGH;
    redState = LOW;
    blinkCount = 3;
  }
    
  digitalWrite(PinRed, redState);
  digitalWrite(PinGreen, greenState);
  digitalWrite(PinBlue, blueState);
}


// The main program will print the blink count
// to the Arduino Serial Monitor
void loop(void) {
  unsigned long blinkCopy;  // holds a copy of the blinkCount

  // to read a variable which the interrupt code writes, we
  // must temporarily disable interrupts, to be sure it will
  // not change while we are reading.  To minimize the time
  // with interrupts off, just quickly make a copy, and then
  // use the copy while allowing the interrupt to keep working.
  noInterrupts();
  blinkCopy = blinkCount;
  interrupts();

  Serial.print("blinkCount = ");
  Serial.println(blinkCopy);
  delay(100);
}
