// Create an IntervalTimer object 
IntervalTimer myTimer;

const int PinRed = A6;  // RED
const int PinGreen = A7;  // GREEN
const int PinBlue = A8;  // BLUE
void setup(void) {
  pinMode(PinRed, OUTPUT);
  pinMode(PinGreen, OUTPUT);
  pinMode(PinBlue, OUTPUT);
  Serial.begin(9600);
  myTimer.begin(blinkLED, 10000);  // blinkLED to run every 1 seconds
}


// initialize the states of LED in analog value ( 0 ~ 255)
int redState = 255;
int blueState = 255;
int greenState = 255;
int fadeAmount = 1;
int count = 1;
int reverse = 0;


// use hard code to make LED light from Red to nothing to Green to nothing to Blue to nothing
// and read the potentialmeter value to change its diming of LED lights (reading from A2)
void blinkLED(void) {
  int val = analogRead(2);
  fadeAmount = val/4;
 
  if (reverse == 0 && count == 1) {
    redState -= fadeAmount;
    if(redState == 0){
      reverse = 1;
    }
  } else if(reverse == 1 && count == 1){
    redState += fadeAmount;
    if(redState == 255){
      reverse = 0;
      count++;
    }
  }else if(reverse == 0 && count == 2){
    blueState -= fadeAmount;
    if(blueState == 0){
      reverse = 1;
    }
  }else if(reverse == 1 && count == 2){
    blueState += fadeAmount;
    if(blueState == 255){
      reverse = 0;
      count++;
    }
   }else if(reverse == 0 && count == 3){
    greenState -= fadeAmount;
    if(greenState == 0){
      reverse = 1;  
    }
  }else{
    greenState += fadeAmount;
    if(greenState == 255){
      reverse = 0;
      count = 1;
    }
  }
    
 
  // analogWrite to make LED light
  analogWrite(PinRed, redState);
  analogWrite(PinGreen, greenState); 
  analogWrite(PinBlue, blueState);
  

  
  
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
  blinkCopy = count;
  interrupts();

  Serial.print("blinkCount = ");
  Serial.println(blinkCopy);
  delay(100);
}
