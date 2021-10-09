#include <cassert>
#include <cstdlib>
#include <cmath>
#include <algorithm>
#include <wiringPi.h>
#include "StepperMotor.hpp"

using namespace std;

// Switching sequence for the 28BYJ48 (clockwise)
static const unsigned SEQUENCE[8][4] = { {1, 0, 0, 0},
                                         {1, 1, 0, 0},
                                         {0, 1, 0, 0},
                                         {0, 1, 1, 0},
                                         {0, 0, 1, 0},
                                         {0, 0, 1, 1},
                                         {0, 0, 0, 1},
                                         {1, 0, 0, 1} };

// stepAngle = (Step angle / gear reduction ratio) = (5.625 / 63.68395)
static const float stepAngle = 0.0883268076179f;


// Default constructor
StepperMotor::StepperMotor() {
    running = false;
    threshold = 0;
    current_pos = 0;
    sequence.resize(8);
    for(unsigned i = 0; i < sequence.size(); i++)
        sequence.at(i).resize(4);
    for(unsigned i = 0; i < sequence.size(); i++)
        for(unsigned j = 0; j < sequence.at(i).size(); j++)
            sequence.at(i).at(j) = SEQUENCE[i][j];
}


// Returns the number of steps associated to a certain angle
unsigned StepperMotor::getSteps(unsigned angle) const {
    return (unsigned) roundf(angle / stepAngle);
}


// Sets the GPIO outputs needed by inputs of the stepper motor driver (ULN2003APG)
// For more details concerning the wiringPi GPIO table conversion refere here:
// http://wiringpi.com/pins/
void StepperMotor::setGPIOutputs(unsigned in1, unsigned in2, unsigned in3, unsigned in4) {
    this->in1 = in1;
    pinMode(in1, OUTPUT);
    this->in2 = in2;
    pinMode(in2, OUTPUT);
    this->in3 = in3;
    pinMode(in3, OUTPUT);
    this->in4 = in4;
    pinMode(in4, OUTPUT);
}


// Sets a maximum threshold angle for the motor rotation
void StepperMotor::setThreshold(unsigned threshold) {
    assert(threshold < 180);
    this->threshold = threshold;
}


// Runs the stepper motor.
// * direction: 1 to go clockwise, -1 to go counterclockwise
// * angle: can assume values from 0 to 360 degrees
// * speed: from 20% (minimum speed) to 100% (maximum speed)
void StepperMotor::run(int direction, unsigned angle, unsigned speed) {
    float td;
    unsigned nsteps, count, ndegrees;

    running = true;

    // Check the direction and angle values
    assert(direction == 1 || direction == -1);
    assert(angle <= 360);

    // Check the speed value (5 speed modes allowed, from 20% to 100%)
    switch(speed) {
        case(20): break;
        case(40): break;
        case(60): break;
        case(80): break;
        case(100): break;
        default: return;
    }

    // Delay between each step of the switching sequence (in microseconds)
    td = (5 * 100 / (float) speed) * 1000;

    // Set the right number of steps to do, taking in account of the threshold
    if(abs(current_pos + direction * angle) > threshold && threshold != 0)
        ndegrees = threshold - direction * current_pos;
    else ndegrees = angle;

    nsteps = getSteps(ndegrees);

    // To go counterclockwise we need to reverse the switching sequence
    if(direction == -1)
        reverse(sequence.begin(), sequence.end());

    count = 0;
    for(unsigned i = 0; i < nsteps; i++) {
        if(count == 8)
            count = 0;

        if(sequence.at(count).at(0) == 1)
            digitalWrite(in1, HIGH);
        else digitalWrite(in1, LOW);

        if(sequence.at(count).at(1) == 1)
            digitalWrite(in2, HIGH);
        else digitalWrite(in2, LOW);

        if(sequence.at(count).at(2) == 1)
            digitalWrite(in3, HIGH);
        else digitalWrite(in3, LOW);

        if(sequence.at(count).at(3) == 1)
            digitalWrite(in4, HIGH);
        else digitalWrite(in4, LOW);

        count++;
        delayMicroseconds(td); // minimum delay 5ms (speed 100%), maximum delay 25ms (speed 20%)
    }

    // Cleanup (recommended in order to prevent stepper motor overheating)
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);

    // Reverse again in order to restore the original vector for the next operations
    if(direction == -1)
        reverse(sequence.begin(), sequence.end());

    // Update the state
    this->nsteps += nsteps;
    current_pos += direction * ndegrees;
    running = false;
}


// Sends to sleep the stepper motor for a certain amount of time (in milliseconds)
void StepperMotor::wait(unsigned milliseconds) const {
    delay(milliseconds);
}
