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
GPIO.setup(hallEffect_pin, GPIO.IN)
GPIO.setup(button_pin, GPIO.IN, GPIO.PUD_UP)
GPIO.setup(servoF_pin, GPIO.OUT)
GPIO.setup(servoR_pin, GPIO.OUT)
GPIO.setup(led_pins, GPIO.OUT)

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

# Registers
power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c
accel_config = 0x1c
gyro_x = 0x43
gyro_y = 0x45
gyro_z = 0x47
accel_x = 0x3b
accel_y = 0x3d
accel_z = 0x3f

 
##########################################################
    #ACCEL SETUP
##########################################################
def read_value(reg):
    # Reads register value at address
    h = bus.read_value_byte_data(address, reg)
    l = bus.read_value_byte_data(address, reg+1)
    value = (h << 8) + l
    return value
 
def read_value_2c(reg):
    # Reads register value at address
    val = read_value(reg)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val

def read_value_byte(reg):
    # Reads register value at address, whole byte
    return bus.read_value_byte_data(address, reg)

def dist(a,b):
    return math.sqrt((a*a)+(b*b))
 
def get_y_incline(x,y,z):
    # Does trigernometry to get angle of rotation from gravity
    radianValue = math.atan2(x, dist(y,z))
    return -math.degrees(radianValue)
 
def get_x_incline(x,y,z):
    # Does trigernometry to get angle of rotation from gravity
    radianValue = math.atan2(y, dist(x,z))
    return math.degrees(radianValue)
 
bus = smbus.SMBus(1) # bus = smbus.SMBus(0) for Revision 1
address = 0x68       # via i2cdetect
 
# Activate, to be able to address the module
bus.write_byte_data(address, power_mgmt_1, 0)
# set accelerometer to +-16g
bus.write_byte_data(address, accel_config, 24)
##########################################################
    #END OF ACCEL SETUP
##########################################################
  
##########################################################
    #MISC
##########################################################
endLoop = False

def handler(signum, frame):
    # Ends program
    print("Exit pressed")
    global endLoop
    endLoop = True
    
signal.signal(signal.SIGINT, handler) #handle ctrl c to close log file
##########################################################
    #END OF MISC
##########################################################

##########################################################
    #VAR INIT
##########################################################
averageCount = 0 #Every poll increments counter
accelTotal_x = 0 #Sum of accel x to average over frame
accelTotal_y = 0 #Sum of accel y to average over frame
accelTotal_z = 0 #Sum of accel z to average over frame
updateInterval = 0.02 #in seconds. The polling interval
outputInterval = 0.5 #in seconds. How long a frame is
outputIntervalCount = outputInterval / updateInterval

bumpCount = 0 #Sum of bump count to average over frame
hallEffectCount = 0 #Sum of hall to average over frame
angleTotal = 0 # To average out angle value as it's quite erratic with bumps

bumpThreshold = 0.5 #How many bumps in an interval
bumpLimit = 3.5 * 9.81 #in m/s/s Previous g value +- this in m/s/s
cadenceThreshold = ((40 * 4) / 60) * outputInterval # Minimum pedalling RPM.
angleThreshold = -5 # In degrees, 0 is flat, -ve is downhill. Sorta deprecated.
previousAccel = 1.0 # The previous acceleration value you compare the current one to
##########################################################
    #END OF VAR INIT
##########################################################

##########################################################
    #BUTTON/HALL HANDLING
##########################################################
def hallEffectTriggered(channel):
    global hallEffectCount
    hallEffectCount = hallEffectCount + 1

#When the hall effect sensor is triggered, run hallEffectTriggered
GPIO.add_event_detect(hallEffect_pin, GPIO.FALLING, callback = hallEffectTriggered, bouncetime = 50)

systemState = "auto"
def buttonTriggered(channel):
    global systemState
    if systemState == "auto":
        systemState = "locked"

    elif systemState == "locked":
        systemState = "unlocked"

    else:
        systemState = "auto"


#When the button sensor is triggered, run hallEffectTriggered
GPIO.add_event_detect(button_pin, GPIO.FALLING, callback = buttonTriggered, bouncetime = 1000)
##########################################################
    #END OF BUTTON/HALL HANDLING
##########################################################

##########################################################
    #SERVO OUTPUT
##########################################################
locked = False
def lock():
    global locked
    if not locked:
        #Go to locked position
        servoF.ChangeDutyCycle(2)
        servoR.ChangeDutyCycle(2)
        time.sleep(0.5)
        #Turn off
        servoF.ChangeDutyCycle(0)
        servoR.ChangeDutyCycle(0)
        locked = True
    
def unlock():
    global locked
    if locked:
        #Go to unlocked position
        servoF.ChangeDutyCycle(5)
        servoR.ChangeDutyCycle(6)
        time.sleep(0.5)
        #Turn off
        servoF.ChangeDutyCycle(0)
        servoR.ChangeDutyCycle(0)
        locked = False
##########################################################
    #END OF SERVO OUTPUT
##########################################################
    
##########################################################
    #MAIN LOOP
##########################################################
while not endLoop:
    #os.system('clear')
    #print("gyroscope")
    #print("--------")

    #Set LED colour to system mode
    if systemState == "locked":
        ledR.ChangeDutyCycle(100)
        ledG.ChangeDutyCycle(0)
        ledB.ChangeDutyCycle(0)
        
        lock()
    elif systemState == "unlocked":
        ledR.ChangeDutyCycle(0)
        ledG.ChangeDutyCycle(0)
        ledB.ChangeDutyCycle(100)
        
        unlock()
    else:
        ledR.ChangeDutyCycle(0)
        ledG.ChangeDutyCycle(100)
        ledB.ChangeDutyCycle(0)
        
        #Read values from acceleromter/gyro
        gyro_xout = read_value_2c(gyro_x)
        gyro_yout = read_value_2c(gyro_y)
        gyro_zout = read_value_2c(gyro_z)
        #print ("gyro_xout: ", ("%5d" % gyro_xout), " scaled: ", (gyro_xout / 131))
        #print ("gyro_yout: ", ("%5d" % gyro_yout), " scaled: ", (gyro_yout / 131))
        #print ("gyro_zout: ", ("%5d" % gyro_zout), " scaled: ", (gyro_zout / 131))
         
        #print ("")
        #print ("accelerometer")
        #print ("---------------------")
        accel_xout = read_value_2c(accel_x)
        accel_yout = read_value_2c(accel_y)
        accel_zout = read_value_2c(accel_z)
        # 2g is 0/16384, 4 is 1/8192, 8 is 2/4096, 3/2048
        accel_xout_scaled = accel_xout / 2048.0
        accel_yout_scaled = accel_yout / 2048.0
        accel_zout_scaled = accel_zout / 2048.0
        
        accelTotal_x = accelTotal_x + accel_xout_scaled
        accelTotal_y = accelTotal_y + accel_yout_scaled
        accelTotal_z = accelTotal_z + accel_zout_scaled
        
        #print ("accel_xout: ", ("%6d" % accel_xout), " scaled: ", accel_xout_scaled)
        #print ("accel_yout: ", ("%6d" % accel_yout), " scaled: ", accel_yout_scaled)
        #print ("accel_zout: ", ("%6d" % accel_zout), " scaled: ", accel_zout_scaled)
         
        x_rotation = get_x_incline(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled)
        y_rotation = get_y_incline(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled)
        
        #angleTotal = angleTotal + y_rotation

        # Creates upper and lower limits from previous value +- limit
        accelLowerValue = (previousAccel * 9.81) - bumpLimit
        accelHigherValue = (previousAccel * 9.81) + bumpLimit
        
        # If current reading exceeds either limit - is a bump
        if (accel_zout_scaled * 9.81) < accelLowerValue or (accel_zout_scaled * 9.81) > accelHigherValue:
            bumpCount = bumpCount + 1
        #Update previous reading value
        previousAccel = accel_zout_scaled
        
        #Every number of periods, average values over a longer period
        averageCount = averageCount + 1
        if averageCount == outputIntervalCount:
            #Average all values, control servos accordingly

            # Turning rotation into a multiplier
            # Average angle over a frame
            angleTotal = angleTotal / outputIntervalCount
            angleMultiplier = angleTotal + 90
            #-90 (downhill) becomes 0, 0 (flat) becomes 1, 90 (uphill) becomes 2
            angleMultiplier = angleMultiplier / 90
            bumpThreshold = bumpThreshold * angleMultiplier

            #if smooth enough and everything else ideal
            #if bumpCount < bumpThreshold and hallEffectCount >= cadenceThreshold and y_rotation > angleThreshold:
            if bumpCount < bumpThreshold and hallEffectCount >= cadenceThreshold:
                lock()
                isLocked = True
            else:
                unlock()
                isLocked = False
            #accelTotal_x = accelTotal_x / outputIntervalCount
            #accelTotal_y = accelTotal_y / outputIntervalCount
            #accelTotal_z = accelTotal_z / outputIntervalCount
            #print ("average x accel: ", (accelTotal_x))
            #print ("average y accel: ", (accelTotal_y))
            #print ("average z accel: ", (accelTotal_z))
            #print ("average X Rotation: " , x_rotation)
            #print ("average Y Rotation: " , y_rotation)
            #print ("angle multiplier: " , angleMultiplier)
            
            #print ("Number of rotations: " , hallEffectCount)
            #print ("Number of bumps: " , bumpCount)
            #print ("Suspension locked: " , isLocked) 
            
            # Reset counters/values
            averageCount = 0
            accelTotal_x = 0
            accelTotal_y = 0
            accelTotal_z = 0
            angleTotal = 0
            hallEffectCount = 0
            bumpCount = 0
        time.sleep(updateInterval)
##########################################################
    #END OF MAIN LOOP
##########################################################

print("Exiting...")
GPIO.remove_event_detect(hallEffect_pin)
GPIO.remove_event_detect(button_pin)
servoF.stop()
servoR.stop()
GPIO.cleanup()
