// Including necessary libraries
#include <iostream>
#include <ros/ros.h>
#include <stdlib.h>
#include <std_msgs/Int32.h>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>

using namespace std;

// g++ -Wall -Wextra -Werror -o keyboard_test keyboard_control.cpp

int ANGLE_STEP = 30;

class Keyboard
{

private:

    // ROS Attributes
    ros::NodeHandle nh;
    ros::Publisher pub_angle;
    ros::Publisher pub_speed_cw;
    ros::Publisher pub_speed_ccw;
    
    // ROS Messages
    std_msgs::Int32 angle_msg;
    std_msgs::Int32 cw_motor_msg;
    std_msgs::Int32 ccw_motor_msg;

    // Encapsulation functions
    int getch(void);

    char key;

public:
    Keyboard();
    void send_commands(void);
};

Keyboard::Keyboard()
{
    this->pub_angle     = nh.advertise<std_msgs::Int32>("/servo_angle",1);
    this->pub_speed_cw  = nh.advertise<std_msgs::Int32>("/CW/motor_speed",1);
    this->pub_speed_ccw = nh.advertise<std_msgs::Int32>("/CCW/motor_speed",1);

    angle_msg.data = 0;
    cw_motor_msg.data = 0;
    ccw_motor_msg.data = 0;
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
        if (angle_msg.data<=180 - ANGLE_STEP )
        {
            angle_msg.data+=ANGLE_STEP;
            this->pub_angle.publish(angle_msg);
            ROS_INFO_STREAM("Angle = " << angle_msg.data);
        }
        else
        {
            ROS_INFO_STREAM("Angle has reached MAX value! Do not press W anymore!");
        }    
    
    }
    else if (key=='s' || key=='S')
    {
        if (angle_msg.data>=ANGLE_STEP)
        {
            angle_msg.data-=ANGLE_STEP;
            this->pub_angle.publish(angle_msg);
            ROS_INFO_STREAM("Angle = " << angle_msg.data);
        }
        else
        {
            ROS_INFO_STREAM("Angle has reached MIN value! Do not press S anymore!");
        } 

    }
    else if (key=='e' || key=='E')
    {
        ccw_motor_msg.data++;
        ROS_INFO_STREAM("CCW Speed = " << ccw_motor_msg.data);
    }
    else if (key=='d' || key=='D')
    {
        ccw_motor_msg.data--;
        ROS_INFO_STREAM("CCW Speed = " << ccw_motor_msg.data);
    }
    else if (key=='q' || key=='Q')
    {
        cw_motor_msg.data++;
        ROS_INFO_STREAM("CW Speed = " << cw_motor_msg.data);
    }
    else if (key=='a' || key=='A')
    {
        cw_motor_msg.data--;
        ROS_INFO_STREAM("CW Speed = " << cw_motor_msg.data);
    }
    else if (key=='x' || key=='X')
    {
        cw_motor_msg.data = 0;
        ccw_motor_msg.data = 0;
        ROS_INFO_STREAM("Stopping motors");
    }
    if (key == '\x03')
      {
        cw_motor_msg.data = 0;
        ccw_motor_msg.data = 0;
        ROS_INFO_STREAM("\n\n                 .     .\n              .  |\\-^-/|  .    \n             /| } O.=.O { |\\\n\n                 CH3EERS\n\n");
        ros::shutdown();
      }
    
    this->pub_speed_cw.publish(cw_motor_msg);
    this->pub_speed_ccw.publish(ccw_motor_msg);

}

int main(int argc, char** argv)
{
    // Init ROS node
    ros::init(argc, argv, "publish_with_keyboard");

    Keyboard keyboard_commands;

    ros::Rate loop_rate(10);

    while (ros::ok())
    {

        keyboard_commands.send_commands();

        loop_rate.sleep();
		ros::spinOnce();
    }
    
}