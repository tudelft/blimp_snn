#ifndef DCMOTORS_HPP
#define DCMOTORS_HPP

// Compile with:
// g++ -Wall -Wextra -Werror -o test dcmotors.cpp -lwiringPi

// Including necessary libraries
#include <wiringPi.h>
#include <iostream>

class Motor{

private:

    // Attributes
    const int _pwmPin;
    const int _dirPin;
    const int _max_speed;

    void init_io();
    void correctSpeed(int &speed);

public:

    Motor(const int pwmPin, const int dirPin);
    void setSpeed(int &speed);

};

#endif