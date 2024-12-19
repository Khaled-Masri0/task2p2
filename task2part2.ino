#include <Wire.h>
#include <LiquidCrystal.h>
#include <Adafruit_ADXL345_U.h> 
#include <math.h> 

const int In1 = 9;    
const int In2 = 10;    
const int motorEnable = 8; 
const int C1 = 2; 
const int C2 = 3; 


volatile int encoderTicks = 0; 
int lastEncoderStateC1 = LOW; 
unsigned long lastPrintTime = 0; 


LiquidCrystal lcd(11, 12, 4, 5, 6, 7);


Adafruit_ADXL345_Unified adxl = Adafruit_ADXL345_Unified(12345);

float previousAngle = 0; 

void setup() {

  pinMode(rIn1, OUTPUT);
  pinMode(In2, OUTPUT);
  pinMode(motorEnable, OUTPUT);

  pinMode(C1, INPUT);
  pinMode(C2, INPUT);

  Serial.begin(9600);


  lcd.begin(16, 2);  
  lcd.print("Starting...");
  delay(2000); 
  lcd.clear();


  attachInterrupt(digitalPinToInterrupt(encoderC1), encoderISR, CHANGE);

  digitalWrite(In1, LOW);
  digitalWrite(In2, LOW);
  analogWrite(motorEnable, 0); 
  adxl.setRange(ADXL345_RANGE_2_G); 
}

void loop() {
  sensors_event_t event;
  adxl.getEvent(&event);

 
  float x = event.acceleration.x;
  float y = event.acceleration.y;
  float z = event.acceleration.z;

  float currentAngle = atan2(x, sqrt(y * y + z * z)) * 180 / M_PI; 
  float angleDifference = currentAngle - previousAngle; 
  previousAngle = currentAngle; 

  Serial.print("Current angle: ");
  Serial.println(currentAngle);
  Serial.print("Angle difference: ");
  Serial.println(angleDifference);

  lcd.setCursor(0, 0);
  lcd.print("MPU:");
  lcd.print(currentAngle, 1);
  lcd.print("Enc:");
  lcd.print(encoderTicks);

  lcd.setCursor(0, 1);
  lcd.print("Diff:");
  lcd.print(angleDifference, 1);
  lcd.print("      "); 

  if (angleDifference > 0) {
    moveMotor(angleDifference, true); 
  } else if (angleDifference < 0) {
    moveMotor(abs(angleDifference), false); 
  }

  delay(30); 
}

void moveMotor(float angle, bool forward) {
  int moveDuration = map(angle, 0, 360, 0, 2000); 

  if (forward) {
    digitalWrite(In1, HIGH);
    digitalWrite(In2, LOW);
  } else {
    digitalWrite(In1, LOW);
    digitalWrite(In2, HIGH);
  }

  analogWrite(motorEnable, 150); 
  delay(moveDuration); 

  stopMotor(); 
}

void stopMotor() {
  digitalWrite(In1, LOW);
  digitalWrite(In2, LOW);
  analogWrite(motorEnable, 0); 
}

void encoderISR() {
  int c1State = digitalRead(C1);
  int c2State = digitalRead(C2);

  if (c1State != lastEncoderStateC1) {
    if (c2State != c1State) {
      encoderTicks++; 
    } else {
      encoderTicks--; 
    }
    lastEncoderStateC1 = c1State;
  }
}