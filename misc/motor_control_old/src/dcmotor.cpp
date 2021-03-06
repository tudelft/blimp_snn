#include "dcmotor.hpp"

using namespace std;

// g++ -Wall -Wextra -Werror -o test dcmotors.cpp -lwiringPi

/// Global variables
int MAX_SPEED = 10;
int CLOCK = 2;
int PWM_RANGE = 45; // 19.2 MHz / 2 / 100 = 20 kHz

/// GPIOs initialization
void Motor::init_io()
{
    wiringPiSetupGpio();
    pinMode(_pwmPin,PWM_OUTPUT);

    pwmSetMode(PWM_MODE_MS);
    pwmSetRange(PWM_RANGE);
    pwmSetClock(CLOCK);

    pinMode(_dirPin,OUTPUT);
}

/**
 * Apply speed correction: from speed [0-10] to speed [25-86]
 *
 * @param[in] speed:  speed selected by user (between 0, min, and 10, max)
 * @param[out] speed: corrected speed (between 25 and 86) corresponding to the PWM duty cycle % to be send to the driver
 */
void Motor::correctSpeed(int &speed){
    switch (speed)
    {
    case 0 : speed = 0;  break;
    case 1 : speed = 10; break;//25; break;
    case 2 : speed = 13; break;//32; break;
    case 3 : speed = 17; break;//39; break;
    case 4 : speed = 20; break;//45; break;
    case 5 : speed = 23; break;//52; break;
    case 6 : speed = 27; break;//59; break;
    case 7 : speed = 30; break;//66; break;
    case 8 : speed = 33; break;//72; break;
    case 9 : speed = 37; break;//79; break;
    case 10: speed = 40; break;//86; break;
    }
}

/**
 * Motor object initialization: default constructor.  
 * CAREFUL!! -> Each DC motor should be connected to the different PWMx or they will spin in the same direction!! (keep reading) 
 * NOTE: There are only two hardware PWM signals on the Raspberry Pi:  
 * PWM0: GPIOs 13, 19 and PWM1: GPIOs 12 and 18.
 *
 * @param[in] pwmPin: PWM signal control pin definition (either 13, 19 [PWM0] or 12, 18 [PWM1])
 * @param[in] dirPin: spinning direction control pin definition
 * @param[out] _pwmPin: PWM signal control pin definition assigned to _pwmPin attribute
 * @param[out] _dirPin: spinning direction control pin definition assigned to _pwmDir attribute
 * @param[out] _max_speed: maximum speed defined for the user assigned to _max_speed attribute
 */
Motor::Motor()
{
    bool pwmPin_ok = ros::param::get("~pwmPin", _pwmPin);
    bool dirPin_ok = ros::param::get("~dirPin", _dirPin);
    _max_speed = MAX_SPEED;
    init_io();

    this->speed_sub = nh.subscribe("motor_speed",1000,&Motor::setSpeed,this);
}

/**
 * Set speed and direction at which the DC motor will rotate (user speed interval: [0-10])
 *
 * @param[in] speed:  speed selected by user (between 0, min, and 10, max)
 * @param[out] speed: appropriate value of the speed to be sent to the DC motor [25-86]
 */
void Motor::setSpeed(const std_msgs::Int32& speed_msg)
{
    //ROS_INFO_STREAM("speed_msg = " << speed_msg.data);

        int dir_value;
        int speed = speed_msg.data;

    if (speed_msg.data < 0)
    {
        speed = -speed_msg.data;
        dir_value = 1;
    }

    else
    {
        dir_value = 0;
    }

    if (speed_msg.data > _max_speed)
    {
        speed = _max_speed;
    }
    
    correctSpeed(speed);

    //ROS_INFO_STREAM("corrected speed = " << speed);
    //ROS_INFO_STREAM("direction pin = " << _dirPin);
    //ROS_INFO_STREAM("pwm pin = " << _pwmPin);

    digitalWrite(_dirPin, dir_value);
    pwmWrite(_pwmPin, speed);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "subscribe_to_speed");

    Motor motor;
    ros::spin();
}

    /*
    Motor motor1(19, 26);
    Motor motor2(12, 21);

    float i = 0;
    int j = 0;
    int k = 0;
    while(true)
    {

        //float speed = 0;

        j = (int)i;
        motor1.setSpeed(k);
        motor2.setSpeed(j); delay(5);

        i += 0.001;

        cout << "j= " << j << endl;
        cout << "k = " << k << endl;

    }
*/