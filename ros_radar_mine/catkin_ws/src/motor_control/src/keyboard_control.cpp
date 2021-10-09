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

int MAX_ANGLE  = 10;
int MAX_SPEED  = 10;

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

    command_msg.angle = 0;
    command_msg.cw_speed = 0;
    command_msg.ccw_speed = 0;
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
        if (command_msg.angle < MAX_ANGLE)
        {
            command_msg.angle++;
            //this->pub_angle.publish(angle_msg);
            ROS_INFO_STREAM("Angle = " << command_msg.angle);
        }
        else
        {
            ROS_INFO_STREAM("Angle has reached MAX value! Do not press W anymore!");
        }    
    
    }
    else if (key=='s' || key=='S')
    {
        if (command_msg.angle > 0)
        {
            command_msg.angle--;
            //this->pub_angle.publish(angle_msg);
            ROS_INFO_STREAM("Angle = " << command_msg.angle);
        }
        else
        {
            ROS_INFO_STREAM("Angle has reached MIN value! Do not press S anymore!");
        } 

    }
    else if (key=='e' || key=='E')
    {   
        if (command_msg.ccw_speed < MAX_SPEED)
        {
            command_msg.ccw_speed++;
        }
        ROS_INFO_STREAM("CCW Speed = " << command_msg.ccw_speed);
    }
    else if (key=='d' || key=='D')
    {
        if (command_msg.ccw_speed > 0)
        {
            command_msg.ccw_speed--;
        }
        ROS_INFO_STREAM("CCW Speed = " << command_msg.ccw_speed);
    }
    else if (key=='q' || key=='Q')
    {
        if (command_msg.cw_speed < MAX_SPEED)
        {
            command_msg.cw_speed++;
        }
        ROS_INFO_STREAM("CW Speed = " << command_msg.cw_speed);
    }
    else if (key=='a' || key=='A')
    {
        if (command_msg.cw_speed > 0)
        {
            command_msg.cw_speed--;
        }
        ROS_INFO_STREAM("CW Speed = " << command_msg.cw_speed);
    }
    else if (key=='x' || key=='X')
    {
        command_msg.cw_speed = 0;
        command_msg.ccw_speed = 0;
        ROS_INFO_STREAM("Stopping DC motors");
    }
    else if (key=='m' || key=='M')
    {
        command_msg.cw_speed = 0;
        command_msg.ccw_speed = 0;
        command_msg.angle = 11;                // CHEEEEEEEEEEEEEEEEEEEEEEEEEECK
        ROS_INFO_STREAM("Stopping ALL motors");
    }
    if (key == '\x03')
      {
        command_msg.cw_speed = 0;
        command_msg.ccw_speed = 0;
        command_msg.angle = 2021;
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