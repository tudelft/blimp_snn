#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Apr 6 11:44:07 2021

@author: marina
"""

class PID:
    
    def __init__(self, P, I, D, dt, simple):
        """
        PID definitions
        
        Parameters
        ----------
        P : Proportional gain
        I : Integral gain
        D : Derivative gain
        dt : Sample time
        """
        self.Kp = P
        self.Ki = I
        self.Kd = D
        
        self.dt = dt
        
        self.heights = 0
        
        if simple:
            self.clear_simple()
        else:
            self.clear()
    
    def clear_simple(self):
        """
        Variables initialization for update_simple()
        
        Parameters
        ----------
        self.integral : Integral term
        self.previous_error : Error at the previous timestep
        self.window_up : Saturation value for the integral term
        """
        self.integral       = 0
        self.previous_error = 0
        self.window_up      = 20
        
    def clear(self):
        """
        Variables initialization for update()
        """
        
        if self.Ki != 0:
            self.Ti = self.Kp/self.Ki
        else:
            self.Ti = 10000000
            
        if self.Kd != 0:
            self.Td = self.Kd/self.Kp
        else:
            self.Td = 0
        
        self.error_tm2 = 0
        self.error_tm1 = 0
        self.u_tm1     = 0
    
    def update_simple(self, error):
        """
        PID implementation based on https://en.wikipedia.org/wiki/PID_controller#Pseudocode
        
        previous_error := 0
        integral := 0
        
        loop:
            error := setpoint − measured_value
            integral := integral + error × dt
            derivative := (error − previous_error) / dt
            output := Kp × error + Ki × integral + Kd × derivative
            previous_error := error
            wait(dt)
            goto loop
        
        """
        
        self.integral += error * self.dt
        
        # Saturate integral
        if self.integral > self.window_up:
            self.integral = self.window_up
        elif self.integral < -self.window_up:
            self.integral = -self.window_up
        
        self.derivative = (error - self.previous_error)/self.dt
        
        self.previous_error = error
        
        u = self.Kp * error + self.Ki * self.integral + self.Kd * self.derivative
        
        return u
    
    def update(self, error):
        """
        PID implementation based on https://en.wikipedia.org/wiki/PID_controller#Discrete_implementation
        """
        
        first_term  = 1 + self.dt/self.Ti + self.Td/self.dt
        second_term = -1 -2*self.Td/self.dt
        third_term  = self.Td/self.dt
        
        u = self.u_tm1 + self.Kp * (first_term * error + second_term * self.error_tm1 + third_term * self.error_tm2)
        
        self.error_tm2 = self.error_tm1
        self.error_tm1 = error
        self.u_tm1 = u
        
        return u
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    