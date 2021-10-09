import RPi.GPIO as GPIO
import time

servoPIN = 17
GPIO.setmode(GPIO.BCM)
GPIO.setup(servoPIN, GPIO.OUT)

p = GPIO.PWM(servoPIN, 50) # GPIO 17 for PWM with 50Hz
p.start(2.5) # Initialization

try:
  while True:
    p.ChangeDutyCycle(2)
    time.sleep(0.3)
    p.ChangeDutyCycle(0)
    time.sleep(0.5)
    p.ChangeDutyCycle(12) 
    time.sleep(0.3)
    p.ChangeDutyCycle(0)
    time.sleep(0.5)
except KeyboardInterrupt:
  p.stop()
  GPIO.cleanup()
