from __future__ import print_function
import time
#import pololu_drv8835_rpi
from pololu_drv8835_rpi import *

# Set up sequences of motor speeds.
test_forward_speeds = list(range(0, MAX_SPEED, 1)) + \
  [MAX_SPEED] * 200 #+ list(range(MAX_SPEED, 0, -1)) + [0]  

test_reverse_speeds = list(range(0, -MAX_SPEED, -1)) + \
  [-MAX_SPEED] * 200 + list(range(-MAX_SPEED, 0, 1)) + [0]  

motor1 = Motor(19, 26)

try:
    motor1.setSpeed(0)
    
    print("Motor 1 forward")
    for s in test_forward_speeds:
        motor1.setSpeed(s)
        print(s)
        time.sleep(0.5)
    
    print("Motor 1 reverse")
    for s in test_reverse_speeds:
        motor1.setSpeed(s)
        print(s)
        time.sleep(1)

finally:
  # Stop the motors, even if there is an exception
  # or the user presses Ctrl+C to kill the process.
  motor1.setSpeed(0)
