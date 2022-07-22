#!/usr/bin/python
import smbus
import datetime
import math
import time
import os
import csv
import signal
import RPi.GPIO as GPIO
hallEffect_pin = 12 #ACTUAL PIN NUMBER (the one in the circle)
button_pin = 29
servoF_pin = 11
servoR_pin = 13
led_pins = [36,38,40]


GPIO.setmode(GPIO.BOARD)

GPIO.setup(servoF_pin,GPIO.OUT)
GPIO.setup(servoR_pin,GPIO.OUT)
GPIO.setup(led_pins, GPIO.OUT)
GPIO.setup(button_pin, GPIO.IN, GPIO.PUD_UP)

servoF = GPIO.PWM(servoF_pin,50) #pin and pulse frequency (50hz)
servoR = GPIO.PWM(servoR_pin,50) #pin and pulse frequency (50hz)
ledR = GPIO.PWM(led_pins[0], 2000)
ledG = GPIO.PWM(led_pins[1], 2000)
ledB = GPIO.PWM(led_pins[2], 2000)

servoF.start(0)
servoR.start(0)
ledR.start(0)
ledG.start(0)
ledB.start(0)

systemState = "idle"
def buttonTriggered(channel):
    #print("button pressed")
    
    global systemState
    
    if systemState == "idle":
        print("position 1")
        systemState = "recording"
        servoR.ChangeDutyCycle(2)
        time.sleep(0.6)
        servoR.ChangeDutyCycle(0)
        servoF.ChangeDutyCycle(2)
        time.sleep(0.6)
        servoF.ChangeDutyCycle(0)
        
        
    else:
        print("position 2")
        systemState = "idle"
        servoR.ChangeDutyCycle(6)
        time.sleep(0.6)
        servoR.ChangeDutyCycle(0)
        servoF.ChangeDutyCycle(5)
        time.sleep(0.6)
        servoF.ChangeDutyCycle(0)
        
        
        


        
#When the button sensor is triggered, run hallEffectTriggered
GPIO.add_event_detect(button_pin, GPIO.FALLING, callback = buttonTriggered, bouncetime = 1500)

while True:
    time.sleep(0.02)

servoF.stop()
servoR.stop()
GPIO.cleanup()
print("finished")



#     if systemState == "idle":
#         print("position 1")
#         systemState = "recording"
#         servoR.ChangeDutyCycle(2)
#         time.sleep(0.8)
#         servoR.ChangeDutyCycle(0)
#         
#         
#     else:
#         print("position 2")
#         systemState = "idle"
#         servoR.ChangeDutyCycle(6)
#         time.sleep(0.8)
#         servoR.ChangeDutyCycle(0)