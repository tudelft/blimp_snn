#!/usr/bin/env python
import pigpio
import time
import rospy
from motor_control.msg import MotorCommand

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
        angle_sub: subscriber to the /servo_angle topic, with motor_control::MotorCommand message type and update_angle as callback function
        """
        self._servoPin = rospy.get_param("~servoPin")
        self.pwm = pigpio.pi()
        self.pwm.set_mode(self._servoPin, pigpio.OUTPUT)
        self.pwm.set_PWM_frequency(self._servoPin, SERVO_FREQUENCY)
        self.angle_sub = rospy.Subscriber("/motor_control", MotorCommand, self.update_angle)
    
    def update_angle(self,msg):
        """
        Function that converts the angle command to the appropriate duty cycle to
        handle the movement of the servo
        """
        if msg.angle > 10:
            #rospy.loginfo("closing servo")
            self.pwm.set_PWM_dutycycle(self._servoPin,0)
            self.pwm.set_PWM_frequency(self._servoPin,0)
        else:
            #correct_angle = 2500-11.1111*msg.angle
            correct_angle = 2500-200*msg.angle
            self.pwm.set_servo_pulsewidth(self._servoPin,correct_angle)
        #time.sleep(0.2)

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