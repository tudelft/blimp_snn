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

class Motor{

private:

    // Attributes
    int _pwmPin;
    int _dirPin;
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
    void setSpeed(const std_msgs::Int32& speed_msg);

};

#endif
