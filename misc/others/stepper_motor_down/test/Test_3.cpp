// Test_3.cpp
//
// Similar to Test_1.cpp, but with two stepper motors
// running in parallel (multithread version).
//

#include <stdlib.h>
#include <pthread.h>
#include <wiringPi.h>
#include "StepperMotor.hpp"

// These functions contain the operations which
// have to be performed by the two stepper motors
void * runStepperMotor1(void *arg);
void * runStepperMotor2(void *arg);

int main() {

    // wiringPi initialization
    wiringPiSetup();

    // Declaration of two thread identifiers
    pthread_t tsm1, tsm2;

    // Declaration of two StepperMotor objects
    StepperMotor sm1, sm2;

    // Declaration of two StepperMotor object pointers
    StepperMotor *psm1, *psm2;

    // Memory allocation
    psm1 = (StepperMotor *) malloc(sizeof(StepperMotor));
    psm2 = (StepperMotor *) malloc(sizeof(StepperMotor));

    // Make a copy of the objects references
    psm1 = &sm1;
    psm2 = &sm2;

    // Launch the threads passing the two pointers as arguments
    pthread_create(&tsm1, NULL, &runStepperMotor1, (void *) psm1);
    pthread_create(&tsm2, NULL, &runStepperMotor2, (void *) psm2);

    // Wait until the threads terminate
    pthread_join(tsm1, NULL);
    pthread_join(tsm2, NULL);

    // Keep alive the main() until all the threads are done
    pthread_exit(NULL);
}


void * runStepperMotor1(void *arg) {

    // Retrieve the argument (copy the reference)
    StepperMotor *psm1 = (StepperMotor *) arg;

    // RPi GPIO | WiringPi
    // -------------------
    // GPIO 17  |    0
    // GPIO 18  |    1
    // GPIO 27  |    2
    // GPIO 22  |    3
    psm1->setGPIOutputs(0, 1, 2, 3);

    // NOTE: Before starting, the current position of both
    // the stepper motors corresponds to 0 degrees

    // Rotate of 90 degrees clockwise at 100% of speed
    psm1->run(1, 90, 100);

    // Sleep for 2 seconds
    psm1->wait(2000);

    // Rotate of 90 degrees counterclockwise at 80% of speed
    psm1->run(-1, 90, 80);

    // Sleep for 1 second
    psm1->wait(1000);

    // Set a threshold of 90 degrees
    psm1->setThreshold(90);

    // Rotate of 180 degrees clockwise at 100% of speed
    // It stops at +90 degrees (because of the threshold)
    psm1->run(1, 180, 100);

    // Sleep for 1 second
    psm1->wait(1000);

    // Rotate of 270 degrees counterclockwise at 60% of speed
    // It stops at -90 degrees (because of the threshold)
    psm1->run(-1, 270, 60);

    // return NULL;
    pthread_exit(NULL);
}


void * runStepperMotor2(void *arg) {

    // Retrieve the argument (copy the reference)
    StepperMotor *psm2 = (StepperMotor *) arg;

    // RPi GPIO | WiringPi
    // -------------------
    // GPIO 23  |    4
    // GPIO 24  |    5
    // GPIO 25  |    6
    // GPIO 4   |    7
    psm2->setGPIOutputs(4, 5, 6, 7);

    // NOTE: Before starting, the current position of both
    // the stepper motors corresponds to 0 degrees

    // Rotate of 90 degrees counterclockwise at 100% of speed
    psm2->run(-1, 90, 100);

    // Sleep for 2 seconds
    psm2->wait(2000);

    // Rotate of 90 degrees clockwise at 80% of speed
    psm2->run(1, 90, 80);

    // Sleep for 1 second
    psm2->wait(1000);

    // Set a threshold of 90 degrees
    psm2->setThreshold(90);

    // Rotate of 180 degrees counterclockwise at 100% of speed
    // It stops at -90 degrees (because of the threshold)
    psm2->run(-1, 180, 100);

    // Sleep for 1 second
    psm2->wait(1000);

    // Rotate of 270 degrees clockwise at 60% of speed
    // It stops at +90 degrees (because of the threshold)
    psm2->run(1, 270, 60);

    // return NULL;
    pthread_exit(NULL);
}
