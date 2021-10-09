#!/usr/bin/env python

"""
# TO DO: 
    1) Check publisher queue size
    2) Call PID params from launch file (parameters)
    3) self.pub_msg.angle, automate the 1 or 10 (min, max) based on sign (same for keyboard_simple)
"""

import time
import rospy
import myPID as PID
import torch
import numpy as np
import ann_automate as ann
#import snn_automate as snn
import snn_fed as snn
# Subscriber messages
from radar_targets_msgs.msg import MyEventArray
from radar_targets_msgs.msg import MyEvent
from std_msgs.msg import Float32
# Publishing messages
from motor_control.msg import MotorCommand

# Global variables:
FREQUENCY = 3.0

class Controller:
    
    def __init__(self):

        # Subscribers and Publisher
        self.sub_radar = rospy.Subscriber("/radar_filter", MyEventArray, self.callback_radar)
        self.sub_h_ref = rospy.Subscriber("/h_ref", Float32, self.callback_h_ref)
        self.pub_motor = rospy.Publisher("/motor_control", MotorCommand, queue_size = 1)
        self.pub_pid   = rospy.Publisher("/u_pid", Float32, queue_size = 1)
        self.pub_snn   = rospy.Publisher("/u_snn", Float32, queue_size = 1)

        # Messages
        self.pub_msg = MotorCommand()
        self.pub_msg_pid = Float32()
        self.pub_msg_snn = Float32()

        # Some important parameters
        self.h_ref = 0.0
        self.range = 0.0
        self.range_filter = 0.0
        self.error = 0.0

        # Controllers
        # PID
        #self.pid = PID.PID(0.8735822376592428, 0.061441163793195155, 0.0028269842479982966, 0.0333333333, True) # self.pid = PID.PID(P, I, D, dt, simple)
        # ANN -> 1.5, 0, 0.3
        # SNN -> 2, 0, 0.46
        self.pid = PID.PID(5, 0.35, 0.5, 0.0333333333, True) # self.pid = PID.PID(P, I, D, dt, simple)

        # ANN
        #self.network = ann.MyANN()
        #self.network.load_state_dict(torch.load("/home/marina/Pi_Git/ros_radar_mine/catkin_ws/src/motor_control/myANN/ANNPID1.pth"))
        #self.network.eval()
        
        # SNN
        i_dynamics, n_dynamics, c_dynamics, en_dynamics, q = self.initSNN()
        self.network = snn.MySNN(i_dynamics, n_dynamics, c_dynamics, en_dynamics, q)
        self.network.load_state_dict(torch.load("/home/marina/Pi_Git/ros_radar_mine/catkin_ws/src/motor_control/myANN/19_SNN.pth"))
        self.network.eval()
        #print(self.network.linear[1].weight.data)
        #print(self.network.neuron.state_dict())

    def initSNN(self):
        q = thresh = tau_v = tau_v = tau_t = alpha_v = alpha_t = v_rest = duration_refrac = dt = 1
        delay = 0
        i_dynamics      = [dt, alpha_t, tau_t]       
        n_dynamics      = [thresh, v_rest, alpha_v, alpha_t, dt, duration_refrac, tau_v, tau_t] 
        c_dynamics      = [1, dt, delay] 
        en_dynamics     = [-2, 2, 40-1, 2, False]
        return i_dynamics, n_dynamics, c_dynamics, en_dynamics, q

    def callback_h_ref(self, msg):
        self.h_ref = msg.data

    def callback_radar(self, msg):
        """
        Assuming that there's ONLY 1 TARGET
        """
        tmp_filter = msg.range_filter
        tmp_range  = msg.target_events

        if len(tmp_filter) != 0:
            self.range_filter = tmp_filter[0]
        #self.range = tmp_range[0].range

    def update_PID(self):
        u = self.pid.update_simple(self.error)
        return u

    def update_ANN(self):
        inp = torch.tensor([float(self.error)])
        _, u = self.network.forward(inp)
        return u

    def update_SNN(self):
        inp = float(self.error)
        _, _, u = self.network.forward(inp)
        #print(self.network.neuron[2].thresh.data)
        return u

    def update_command(self):
        
        self.error = self.h_ref - self.range_filter

        #u_pid = self.update_PID()
        #u_snn = self.update_ANN()
        u_snn = self.update_SNN()

        u = u_snn * 0.7

        #u = u_pid + u_snn

        # Discretize speeds
        #u *= 2
        #u = round(u)

        self.pub_msg.ts = rospy.get_rostime()

        if u >= 0:
            self.pub_msg.angle = 1
        else:
            u = u*(-1)
            self.pub_msg.angle = 10

        self.pub_msg.cw_speed = u
        self.pub_msg.ccw_speed = u

        #self.pub_msg_pid = u_pid
        self.pub_msg_snn = u_snn

        self.pub_pid.publish(self.pub_msg_pid)
        self.pub_snn.publish(self.pub_msg_snn)

        self.pub_motor.publish(self.pub_msg)

    #def convert_command(self,u):
    #    pass

if __name__ == '__main__':
    rospy.init_node('controller') # Node initialization #, anonymous=True)
    myController = Controller()   # Instantiation of the Controller class
    #rospy.spin()
    r = rospy.Rate(FREQUENCY)

    while not rospy.is_shutdown():
        myController.update_command()
        r.sleep()