#!/usr/bin/env python
import RPi.GPIO as GPIO
import time

SERVO_FREQUENCY = 50

class Servo:

    def __init__(self,servoPin):
        self._servoPin = servoPin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(servoPin, GPIO.OUT)
        self.pwm = GPIO.PWM(servoPin, SERVO_FREQUENCY) # GPIO 17 for PWM with 50Hz
        self.pwm.start(0) # Initialization
    
    def update_angle(self,angle):
        duty_cycle = 2+angle/18
        self.pwm.ChangeDutyCycle(duty_cycle)
        time.sleep(2)

try:
    servo = Servo(17)
    while True:
        servo.update_angle(90)
        servo.update_angle(180)
        servo.update_angle(45)
        servo.update_angle(0)
    
except KeyboardInterrupt:
    servo.pwm.stop()
    GPIO.cleanup()