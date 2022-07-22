#!/usr/bin/python
import smbus
import datetime
import math
import time
from datetime import timedelta
import os
import csv
import signal
import RPi.GPIO as GPIO
hallEffect_pin = 12 #ACTUAL PIN NUMBER (the one in the circle)
button_pin = 29
led_pins = [36,38,40]

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(hallEffect_pin, GPIO.IN)
GPIO.setup(button_pin, GPIO.IN, GPIO.PUD_UP)
GPIO.setup(led_pins, GPIO.OUT)


ledR = GPIO.PWM(led_pins[0], 2000)
ledG = GPIO.PWM(led_pins[1], 2000)
ledB = GPIO.PWM(led_pins[2], 2000)


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
    #LOGGING SETUP
##########################################################
logPath = '/home/pi/Programming/newLogs/'
fileName = logPath + str(len([name for name in os.listdir(logPath) if os.path.isfile(os.path.join(logPath, name))])) + ".csv"

endLoop = False

log = open(fileName,"w", newline="")
logWriter = csv.writer(log)
def handler(signum, frame):
    print("Exit pressed")
    global endLoop
    endLoop = True
    
signal.signal(signal.SIGINT, handler) #handle ctrl c to close log file
##########################################################
    #END OF LOGGING SETUP
##########################################################
 
##########################################################
    #VAR INIT
##########################################################
updateInterval = 0.02 #in seconds
outputInterval = 0.5 #in seconds
outputIntervalCount = outputInterval / updateInterval
averageCount = 0
hallEffectCount = 0
angleTotal = 0
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
GPIO.add_event_detect(hallEffect_pin, GPIO.RISING, callback = hallEffectTriggered, bouncetime = 10)

systemState = "idle"
ledR.ChangeDutyCycle(0)
ledG.ChangeDutyCycle(0)
ledB.ChangeDutyCycle(0)

startTime = 0;
endTime = 0;

def buttonTriggered(channel):
    #print("button pressed")
    
    global systemState
    global startTime
    
    if systemState == "idle":
        startTime = datetime.datetime.now()
        systemState = "recording"
        print("recording")
    else:
        global log
        global endTime
        
        endTime = datetime.datetime.now()
        duration = timedelta.total_seconds(endTime - startTime)
        #print(duration)
        logWriter = csv.writer(log)
        logWriter.writerow(["Duration in seconds", duration])
                            
        systemState = "idle"
        print("idle")
        
        log.close()

#When the button sensor is triggered, run hallEffectTriggered
GPIO.add_event_detect(button_pin, GPIO.FALLING, callback = buttonTriggered, bouncetime = 300)
##########################################################
    #END OF BUTTON/HALL HANDLING
##########################################################

logWriter.writerow(["x accel", "y accel", "z accel","x rotation","y rotation","angle average", "hall effect count"])
print("running...")
##########################################################
    #MAIN LOOP
##########################################################
while not endLoop:
    #os.system('clear')
    #print("gyroscope")
    #print("--------")
    
    ledR.ChangeDutyCycle(0)
    ledG.ChangeDutyCycle(0)
    ledB.ChangeDutyCycle(0)
        
        
    if systemState == "recording":
        averageCount = averageCount + 1
        
        ledR.ChangeDutyCycle(100)
        ledG.ChangeDutyCycle(0)
        ledB.ChangeDutyCycle(0)
        
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
        
        #print ("accel_xout: ", ("%6d" % accel_xout), " scaled: ", accel_xout_scaled)
        #print ("accel_yout: ", ("%6d" % accel_yout), " scaled: ", accel_yout_scaled)
        #print ("accel_zout: ", ("%6d" % accel_zout), " scaled: ", accel_zout_scaled)
         
        x_rotation = get_x_incline(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled)
        y_rotation = get_y_incline(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled)
        angleTotal = angleTotal + y_rotation
        
        angleAverage = 0
        if averageCount == outputIntervalCount:
            angleTotal = angleTotal/outputIntervalCount
            angleAverage = angleTotal
            angleTotal = 0
            averageCount = 0

        #log data
        try:
            logWriter.writerow([accel_xout_scaled, accel_yout_scaled, accel_zout_scaled,x_rotation,y_rotation,angleAverage, hallEffectCount])
        except:
            logPath = '/home/pi/Programming/newLogs/'
            fileName = logPath + str(len([name for name in os.listdir(logPath) if os.path.isfile(os.path.join(logPath, name))])) + ".csv"

            log = open(fileName,"w", newline="")
            logWriter = csv.writer(log)
            logWriter.writerow(["x accel", "y accel", "z accel","x rotation","y rotation","angle average", "hall effect count"])
            logWriter.writerow([accel_xout_scaled, accel_yout_scaled, accel_zout_scaled,x_rotation,y_rotation,angleAverage, hallEffectCount])
        
        
        hallEffectCount = 0
        
    else:
        ledR.ChangeDutyCycle(0)
        ledG.ChangeDutyCycle(100)
        ledB.ChangeDutyCycle(0)
    time.sleep(updateInterval)
##########################################################
    #END OF MAIN LOOP
##########################################################

print("Exiting...")
GPIO.remove_event_detect(hallEffect_pin)
GPIO.remove_event_detect(button_pin)
GPIO.cleanup()
log.close()
