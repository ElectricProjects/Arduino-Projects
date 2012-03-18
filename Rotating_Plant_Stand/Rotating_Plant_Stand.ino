
/* 
Rotating Plant Stand
Rotates one revolution every two days, moving a small amount every half an hour
Based on example byTom Igoe 

 */

#include <Stepper.h>

const int stepsPerRevolution = 200;  // change this to fit the number of steps per revolution

Stepper myStepper(stepsPerRevolution, 8,9,10,11);            

long stepCount = 0;         // number of steps the motor has taken
unsigned long interval = (1800000); // half an hour
long previousMillis = 0;

void setup() {
for (int i=0; i <=11; i++)
  {
  myStepper.step(1);
  stepCount=stepCount+1;
  delay(10);
  }
}

void loop() {
  unsigned long currentMillis = millis();
  if(currentMillis - previousMillis > interval) 
  {
  previousMillis = currentMillis; 
  for (int i=0; i <=11; i++)
  {
  myStepper.step(1);
  stepCount=stepCount+1;
  delay(10);
  }

  }
 
}

