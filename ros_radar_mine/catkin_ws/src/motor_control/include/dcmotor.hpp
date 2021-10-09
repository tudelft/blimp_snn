#ifndef DCMOTORS_HPP
#define DCMOTORS_HPP

// Compile with:
// g++ -Wall -Wextra -Werror -o test dcmotors.cpp -lwiringPi

// Including necessary libraries
#include <wiringPi.h>
#include <iostream>
#include <ros/ros.h>
#include <stdlib.h>
#include <std_msgs/Int32.h>
#include "motor_control/MotorCommand.h"

class Motor{

private:

    // Attributes
    int _cw_pwmPin;
    int _cw_dirPin;
    int _ccw_pwmPin;
    int _ccw_dirPin;
    int _max_speed;
    int _speed;

    void init_io();
    void correctSpeed(int &speed);

    // ROS Attributes
    ros::NodeHandle nh;
    ros::Subscriber speed_sub;
    //std_msgs::Int32 speed_msg;

public:

    Motor();
    void setSpeed(const motor_control::MotorCommand& msg);

};

#endif
