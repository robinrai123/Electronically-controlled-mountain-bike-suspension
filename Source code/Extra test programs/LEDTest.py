import RPi.GPIO as GPIO
import time
import random

leds = [36,38,40]
GPIO.setmode(GPIO.BOARD)


GPIO.setup(leds, GPIO.OUT)

    
pwmR = GPIO.PWM(leds[0], 2000)
pwmG = GPIO.PWM(leds[1], 2000)
pwmB = GPIO.PWM(leds[2], 2000)

pwmR.start(50)
pwmG.start(50)
pwmB.start(0)

time.sleep(2)

pwmR.ChangeDutyCycle(100) # as a percentage

time.sleep(3)

pwmR.ChangeDutyCycle(0)
pwmG.ChangeDutyCycle(100)

time.sleep(3)

pwmG.ChangeDutyCycle(0)
pwmB.ChangeDutyCycle(100)

time.sleep(3)
pwmR.stop()
pwmG.stop()
pwmB.stop()
GPIO.cleanup()

