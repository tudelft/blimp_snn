// Including necessary libraries
#include <iostream>
#include <ros/ros.h>
#include <stdlib.h>
//#include <std_msgs/Int32.h>
#include "motor_control/MotorCommand.h"
#include <stdio.h>
#include <unistd.h>
#include <termios.h>

using namespace std;

// g++ -Wall -Wextra -Werror -o keyboard_test keyboard_control.cpp

int MIN_ANGLE  = 1;
int MAX_ANGLE  = 10;
int MAX_SPEED  = 9;

class Keyboard
{

private:

    // ROS Attributes
    ros::NodeHandle nh;
    ros::Publisher pub_;
   
    // ROS Messages
    motor_control::MotorCommand command_msg;

    // Encapsulation functions
    int getch(void);

    char key;

public:
    Keyboard();
    void send_commands(void);
};

Keyboard::Keyboard()
{
    this->pub_ = nh.advertise<motor_control::MotorCommand>("/motor_control",1);

    command_msg.angle = MIN_ANGLE;
    command_msg.cw_speed = 0;
    command_msg.ccw_speed = 0;
    command_msg.ts = ros::Time::now();
    this->pub_.publish(command_msg);
}

int Keyboard::getch(void)
{
  int ch;
  struct termios oldt;
  struct termios newt;

  // Store old settings, and copy to new settings
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;

  // Make required changes and apply the settings
  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_iflag |= IGNBRK;
  newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
  newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
  newt.c_cc[VMIN] = 1;
  newt.c_cc[VTIME] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &newt);

  // Get the current character
  ch = getchar();

  // Reapply old settings
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

  return ch;
}

void Keyboard::send_commands(){
    
    key = getch();

    if (key=='w' || key=='W')
    {
        command_msg.angle = MAX_ANGLE;
        ROS_INFO_STREAM("Angle = " << command_msg.angle);
    }
    else if (key=='s' || key=='S')
    {
        command_msg.angle = MIN_ANGLE;
        ROS_INFO_STREAM("Angle = " << command_msg.angle);
    }
    else if (key=='e' || key=='E')
    {   
        if (command_msg.ccw_speed < MAX_SPEED || command_msg.ccw_speed < MAX_SPEED)
        {
            command_msg.ccw_speed++;
            command_msg.cw_speed++;
        }
        ROS_INFO_STREAM("Speed = " << command_msg.ccw_speed);
    }
    else if (key=='d' || key=='D')
    {
        if (command_msg.ccw_speed > 0 || command_msg.ccw_speed > 0)
        {
            command_msg.ccw_speed--;
            command_msg.cw_speed--;
        }
        ROS_INFO_STREAM("Speed = " << command_msg.ccw_speed);
    }
    else if (key=='0')
    {
        command_msg.ccw_speed = 0;
        command_msg.cw_speed  = 0;
        ROS_INFO_STREAM("Speed = " << command_msg.ccw_speed);
    }
    else if (key=='1')
    {
        command_msg.ccw_speed = 1;
        command_msg.cw_speed  = 1;
        ROS_INFO_STREAM("Speed = " << command_msg.ccw_speed);
    }
    else if (key=='2')
    {
        command_msg.ccw_speed = 2;
        command_msg.cw_speed  = 2;
        ROS_INFO_STREAM("Speed = " << command_msg.ccw_speed);
    }
    else if (key=='3')
    {
        command_msg.ccw_speed = 3;
        command_msg.cw_speed  = 3;
        ROS_INFO_STREAM("Speed = " << command_msg.ccw_speed);
    }
    else if (key=='4')
    {
        command_msg.ccw_speed = 4;
        command_msg.cw_speed  = 4;
        ROS_INFO_STREAM("Speed = " << command_msg.ccw_speed);
    }
    else if (key=='5')
    {
        command_msg.ccw_speed = 5;
        command_msg.cw_speed  = 5;
        ROS_INFO_STREAM("Speed = " << command_msg.ccw_speed);
    }
    else if (key=='6')
    {
        command_msg.ccw_speed = 6;
        command_msg.cw_speed  = 6;
        ROS_INFO_STREAM("Speed = " << command_msg.ccw_speed);
    }
    else if (key=='7')
    {
        command_msg.ccw_speed = 7;
        command_msg.cw_speed  = 7;
        ROS_INFO_STREAM("Speed = " << command_msg.ccw_speed);
    }
    else if (key=='8')
    {
        command_msg.ccw_speed = 8;
        command_msg.cw_speed  = 8;
        ROS_INFO_STREAM("Speed = " << command_msg.ccw_speed);
    }
    else if (key=='9')
    {
        command_msg.ccw_speed = 9;
        command_msg.cw_speed  = 9;
        ROS_INFO_STREAM("Speed = " << command_msg.ccw_speed);
    }
    else if (key=='m' || key=='M')
    {
        command_msg.cw_speed = 0;
        command_msg.ccw_speed = 0;
        command_msg.angle = MIN_ANGLE;                // CHEEEEEEEEEEEEEEEEEEEEEEEEEECK
        ROS_INFO_STREAM("Stopping ALL motors");
    }
    if (key == '\x03')
      {
        command_msg.cw_speed = 0;
        command_msg.ccw_speed = 0;
        command_msg.angle = MIN_ANGLE;
        ROS_INFO_STREAM("\n\n                 .     .\n              .  |\\-^-/|  .    \n             /| } O.=.O { |\\\n\n                 CH3EERS\n\n");
        ros::shutdown();
      }
    
    command_msg.ts = ros::Time::now();
    this->pub_.publish(command_msg);

}

int main(int argc, char* argv[])
{
    // Init ROS node
    ros::init(argc, argv, "publish_with_keyboard");

    Keyboard keyboard_commands;

    ros::Rate loop_rate(10);

    while (ros::ok())
    {

        keyboard_commands.send_commands();

        ros::spinOnce();
        loop_rate.sleep();
    }
    
}