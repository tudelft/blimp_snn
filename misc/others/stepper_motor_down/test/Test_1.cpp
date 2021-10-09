// Test_1.cpp
//
// A simple test to drive the stepper motor.
//

#include <wiringPi.h>
#include "StepperMotor.hpp"

int main() {
    
    // wiringPi initialization
    wiringPiSetup();

    // StepperMotor object declaration
    StepperMotor sm;

    // RPi GPIO | WiringPi
    // -------------------
    // GPIO 17  |    0
    // GPIO 18  |    1
    // GPIO 27  |    2
    // GPIO 22  |    3
    sm.setGPIOutputs(0, 1, 2, 3);

    // NOTE: Before starting, the current position of the
    // stepper motor corresponds to 0 degrees

    // Rotate of 90 degrees clockwise at 100% of speed
    sm.run(1, 90, 100);

    // Sleep for 2 seconds
    sm.wait(2000);

    // Rotate of 90 degrees counterclockwise at 80% of speed
    sm.run(-1, 90, 80);

    // Sleep for 1 second
    sm.wait(1000);

    // Set a threshold of 90 degrees
    sm.setThreshold(90);

    // Rotate of 180 degrees clockwise at 100% of speed
    // It stops at +90 degrees (because of the threshold)
    sm.run(1, 180, 100);

    // Sleep for 1 second
    sm.wait(1000);

    // Rotate of 270 degrees counterclockwise at 60% of speed
    // It stops at -90 degrees (because of the threshold)
    sm.run(-1, 270, 60);

    return 0;
}
