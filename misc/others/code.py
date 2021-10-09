# Use this example for digital pin control of an H-bridge driver
# like a DRV8833, TB6612 or L298N.

import time
import board
#from board import *
import digitalio
import pulseio
from adafruit_motor import motor

#DELAY = 0.01
#STEPS = 200

# You can use any available GPIO pin on both a microcontroller and a Raspberry Pi.
# The following pins are simply a suggestion. If you use different pins, update
# the following code to use your chosen pins.

# To use with a Raspberry Pi:
#coils = (
#    digitalio.DigitalInOut(board.D19),  # A1
#    digitalio.DigitalInOut(board.D26),  # A2
#    digitalio.DigitalInOut(board.D20),  # B1
#    digitalio.DigitalInOut(board.D21),  # B2
#)

#for coil in coils:
#    coil.direction = digitalio.Direction.OUTPUT
#    print(coil.direction)
#
#motor1 = motor.DCMotor(coils[0], coils[1])
#motor2 = motor.DCMotor(coils[2], coils[3])

pwm_A1 = pulseio.PWMOut(board.D19)
#pwm_A1.duty_cycle = int(66/100*(2**16))
pwm_A1.frequency = 25000
pwm_A2 = pulseio.PWMOut(board.D26)
#pwm_A2.duty_cycle = int(66/100*(2**16))
pwm_A2.frequency = 25000

#help(pwm_A1)

motor1 = motor.DCMotor(pwm_A1,pwm_A2)

i = 0

while(i < 0.99):
    #print("Forwards slow")
    i += 0.1
    motor1.throttle = i
    print("throttle:", motor1.throttle)
    time.sleep(0.5)
























