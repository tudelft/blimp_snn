#!/usr/bin/env python
#import RPi.GPIO as GPIO
import pigpio
import time

SERVO_FREQUENCY = 50

class Servo:

    def __init__(self,servoPin):
        self._servoPin = servoPin
        self.pwm = pigpio.pi()
        self.pwm.set_mode(self._servoPin, pigpio.OUTPUT)
        self.pwm.set_PWM_frequency(self._servoPin, SERVO_FREQUENCY)
    
    def update_angle(self,angle):
        correct_angle = 570+10.3333*angle
        self.pwm.set_servo_pulsewidth(self._servoPin,correct_angle)
        time.sleep(2)

    def turn_off(self):
        self.pwm.set_PWM_dutycycle(self._servoPin, 0)
        self.pwm.set_PWM_frequency(self._servoPin, 0 )

try:
    servo = Servo(17)
    while True:
        servo.update_angle(10)
        servo.update_angle(60)
        servo.update_angle(90)
        servo.update_angle(180)
    
except KeyboardInterrupt:
    servo.turn_off()
