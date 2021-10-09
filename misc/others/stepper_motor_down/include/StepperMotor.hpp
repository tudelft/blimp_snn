// Copyright (c) 2016, Cristiano Urban (http://crish4cks.net)
//
// A simple C++ class created to provide an easily interaction with 
// the geared stepper motor 28BYJ48 through the ULN2003APG driver.
//

#ifndef STEPPER_MOTOR_HPP
#define STEPPER_MOTOR_HPP

#include <vector>

using namespace std;

class StepperMotor {
    public:
        StepperMotor();
        bool isRunning() const { return running; }
        unsigned getThreshold() const { return threshold; }
        int getCurrentPosition() const { return current_pos; }
        unsigned getSteps(unsigned angle) const;
        unsigned getNumOfSteps() const { return nsteps; }
        void setGPIOutputs(unsigned in1, unsigned in2, unsigned in3, unsigned in4);
        void setThreshold(unsigned threshold);
        void run(int direction, unsigned angle, unsigned speed);
        void wait(unsigned milliseconds) const;

    private:
        vector< vector<unsigned> > sequence;   // the switching sequence
        bool running;                          // state of the stepper motor
        unsigned threshold;                    // symmetric threshold in degrees
        int current_pos;                       // current position in degrees
        unsigned nsteps;                       // total number of steps from the beginning
        unsigned in1, in2, in3, in4;           // stepper motor driver inputs
};

#endif
