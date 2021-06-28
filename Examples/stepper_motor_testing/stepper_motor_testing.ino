#include <AccelStepper.h>
#include <MultiStepper.h>

// Define stepper motor connections:
#define dirPin 4
#define stepPin 3

AccelStepper stepper(1, stepPin, dirPin);
void setup() {

  stepper.setMaxSpeed(200);
  // stepper.setSpeed(500); // can set negative to go reverse direction
  stepper.setAcceleration(100);
}

void loop() {
    stepper.moveTo(2000);  // Set the target position
    stepper.runToPosition(); // Go to target position with set dx and dx^2
    stepper.setCurrentPosition(0); // Reset the position to 0
    delay(1000);
  
  
}
