#!/usr/bin/env python
import RPi.GPIO as GPIO
import time
import rospy
from std_msgs.msg import Int32

# Global variables:
SERVO_FREQUENCY = 50

class Servo:
    """
    Servo class definition, which subscribes to the topic /servo_angle to get angle commands for the servomotor
    """
    def __init__(self):
        """
        Servo class default constructor. Its attributes are:
        _servoPin: GPIO pin that sends PWM signals to the servo
        pwm:       object from the RPi.GPIO library that handles the servo properties and movement
        angle_sub: subscriber to the /servo_angle topic, with std_msgs::Int32 message type and update_angle as callback function
        """
        self._servoPin = rospy.get_param("~servoPin")
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self._servoPin, GPIO.OUT)
        self.pwm = GPIO.PWM(self._servoPin, SERVO_FREQUENCY)
        self.pwm.start(0) 
        self.angle_sub = rospy.Subscriber("servo_angle", Int32, self.update_angle)
    
    def update_angle(self,angle):
        """
        Function that converts the angle command to the appropriate duty cycle to
        handle the movement of the servo
        """
        duty_cycle = 2+angle.data/18
        self.pwm.ChangeDutyCycle(duty_cycle)
        time.sleep(0.3)

if __name__ == '__main__':
    rospy.init_node('subscribe_to_angle') # Node initialization #, anonymous=True)
    myServo = Servo()                     # Instantiation of the Servo class
    rospy.spin()

# Garbage (that might be useful later):
'''
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
'''