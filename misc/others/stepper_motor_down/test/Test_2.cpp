// Test_2.cpp
//
// Similar to Test_1.cpp, but with two stepper motors
// running sequentially.
//

#include <wiringPi.h>
#include "StepperMotor.hpp"

int main() {
    
    // wiringPi initialization
    wiringPiSetup();
    
    // Declaration of two StepperMotor objects
    StepperMotor sm1, sm2;

    // RPi GPIO | WiringPi
    // -------------------
    // GPIO 17  |    0
    // GPIO 18  |    1
    // GPIO 27  |    2
    // GPIO 22  |    3
    sm1.setGPIOutputs(0, 1, 2, 3);
    
     // RPi GPIO | WiringPi
    // -------------------
    // GPIO 23  |    4
    // GPIO 24  |    5
    // GPIO 25  |    6
    // GPIO 4   |    7
    sm2.setGPIOutputs(4, 5, 6, 7);

    // NOTE: Before starting, the current position of both
    // the stepper motors corresponds to 0 degrees

    // Rotate of 90 degrees clockwise at 100% of speed
    sm1.run(1, 90, 100);
    sm2.run(1, 90, 100);
    
    // Sleep for 2 seconds (2 x 1 second)
    sm1.wait(1000);
    sm2.wait(1000);
    
    // Rotate of 90 degrees counterclockwise at 80% of speed
    sm1.run(-1, 90, 80);
    sm2.run(-1, 90, 80);
    
    // Sleep for 1 second (2 x 500 milliseconds)
    sm1.wait(500);
    sm2.wait(500);
    
    // Set a threshold of 90 degrees for sm1
    sm1.setThreshold(90);
    
    // Set a threshold of 45 degrees for sm2
    sm2.setThreshold(45);

    // Rotate of 180 degrees clockwise at 100% of speed
    // sm1 stops at +90 degrees (because of the threshold)
    // sm2 stops at +45 degrees (because of the threshold)
    sm1.run(1, 180, 100);
    sm2.run(1, 180, 100);
    
    // Sleep for 1 second
    sm1.wait(500);
    sm2.wait(500);
    
    // Rotate of 270 degrees counterclockwise at 60% of speed
    // sm1 stops at -90 degrees (because of the threshold)
    // sm2 stops at -45 degrees (because of the threshold)
    sm1.run(-1, 270, 60);
    sm2.run(-1, 270, 60);
    
    return 0;
}
