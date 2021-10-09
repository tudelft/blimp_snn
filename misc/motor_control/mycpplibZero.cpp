#include <wiringPi.h>
#include <iostream>

// g++ -Wall -Wextra -Werror -o test mycpplibZero.cpp -lwiringPi

using namespace std;

int _max_speed = 100;  // 19.2 MHz / 2 / 480 = 20 kHz
int MAX_SPEED = _max_speed;

class Motor{

private:

    // Attributes
    int _pwmPin;
    int _dirPin;

public:

    Motor(int &pwmPin, int &dirPin){
        _pwmPin = pwmPin;
        _dirPin = dirPin;
    }

    void setSpeed(int &speed){

        int dir_value;

        if (speed < 0)
        {
            speed = -speed;
            dir_value = 1;
        }
        else
        {
            dir_value = 0;
        }

        if (speed > MAX_SPEED)
        {
            speed = MAX_SPEED;
        }
        
        digitalWrite(_dirPin, dir_value);
        pwmWrite(_pwmPin, speed);
    }

};


int main (void)
{
    wiringPiSetupGpio();
    pinMode(19,PWM_OUTPUT);
    pinMode(26,OUTPUT);

    pwmSetMode(PWM_MODE_MS);
    pwmSetRange(MAX_SPEED);
    pwmSetClock(2);

    int pwmPin = 19;
    int dirPin = 26;

    Motor motor1(pwmPin, dirPin);

    float i = 0;
    int j = 0;
    while(true)
    {

        //float speed = 0;

        j = (int)i;

        motor1.setSpeed(j); delay(1);

        i += 0.005;

        cout << j << endl;

    }
    return 0 ;
}