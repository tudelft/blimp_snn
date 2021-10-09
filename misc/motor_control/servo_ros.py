#!/usr/bin/env python
import RPi.GPIO as GPIO
import time
import rospy
from std_msgs.msg import Int32

SERVO_FREQUENCY = 50

class Servo:

    def __init__(self,servoPin):
        self._servoPin = rospy.get_param("~servoPin")
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(servoPin, GPIO.OUT)
        self.pwm = GPIO.PWM(servoPin, SERVO_FREQUENCY) # GPIO 17 for PWM with 50Hz
        self.pwm.start(0) # Initialization
    
    def update_angle(self,angle):
        duty_cycle = 2+angle.data/18
        self.pwm.ChangeDutyCycle(duty_cycle)
        time.sleep(0.3)


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
     # name are launched, the previous one is kicked off. The
     # anonymous=True flag means that rospy will choose a unique
     # name for our 'listener' node so that multiple listeners can
     # run simultaneously.
     rospy.init_node('subscribe_to_angle')#, anonymous=True)

     rospy.Subscriber("servo_angle", Int32, Servo.update_angle)
 
     # spin() simply keeps python from exiting until this node is stopped
     rospy.spin()
 
 if __name__ == '__main__':
     listener()


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