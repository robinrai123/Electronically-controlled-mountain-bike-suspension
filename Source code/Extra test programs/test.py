import time, datetime, sys
import RPi.GPIO as GPIO
sense_pin = 7
GPIO.setmode(GPIO.BOARD)
GPIO.setup(sense_pin, GPIO.IN)
last_time = time.time()
this_time = time.time()
RPM = 0

def EventsPerTime(channel):
    global RPM, this_time, last_time
    this_time = time.time()
    RPM = (1/(this_time - last_time)) * 60
    print('Current RPM = ','{:7.1f}'.format(RPM))
    last_time = this_time
    return()

GPIO.add_event_detect(sense_pin, GPIO.RISING, callback = EventsPerTime, bouncetime = 1)

timedate = datetime.datetime.now().strftime('%H:%M %Y%m%d %a')
print('System Active @', timedate)
print('Ctrl C to quit')
try:
    for x in range(0,1200):
        time.sleep(0.5)
except:
    time.sleep(2)
    GPIO.remove_event_detect(sense_pin)
    GPIO.cleanup()
    print('Done')