#include <wiringPi.h>
#include <iostream>
#include <softPwm.h>

// g++ -Wall -Wextra -Werror -o servotest servo.cpp -lwiringPi

using namespace std;

int PIN = 17;

int main(){

wiringPiSetupGpio();
pinMode(PIN,PWM_OUTPUT);

//pwmSetMode(PWM_MODE_MS);
//pwmSetClock(192);
//pwmSetRange(2000);  //50 Hz frequency
//pwmSetRange(100);
//pwmSetClock(2);

softPwmCreate(PIN,0,100);


float delay_period = 50;

for (int i = 50; i < 100; i++)
{
    //pwmWrite(17,i);
    delay(delay_period);

    cout << i << endl;

    softPwmWrite (PIN,i);
}

for (int i = 250; i > 50; i--)
{
    pwmWrite(17,i);
    delay(delay_period);
}

return 0;

}